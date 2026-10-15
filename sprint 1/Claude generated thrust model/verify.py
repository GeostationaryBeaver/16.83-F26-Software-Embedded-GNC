"""
Verify the nozzle model against the source hand analysis.

Run after any edit to nozzle_thrust.py:

    python verify.py

Checks the steady-state chain to 4 significant figures and asserts the two
properties that a wrong choke criterion or Cd convention would break.
"""

from __future__ import annotations

import sys

from nozzle_thrust import (
    G0,
    Conditions,
    Model,
    Phase,
    Schedule,
    exit_mach,
    first_choke_pressure,
    simulate,
    steady_state,
    steady_thrust_reference,
)

_failures: list[str] = []


def check(label: str, got: float, expected: float, tol: float, unit: str = "") -> None:
    ok = abs(got - expected) <= tol
    status = "ok  " if ok else "FAIL"
    print(
        "  %s %-26s %12.4f  expected %10.4f %s"
        % (status, label, got, expected, unit)
    )
    if not ok:
        _failures.append(label)


def check_true(label: str, ok: bool, detail: str = "") -> None:
    print("  %s %-26s %s" % ("ok  " if ok else "FAIL", label, detail))
    if not ok:
        _failures.append(label)


def main() -> int:
    model = Model()
    noz, gas, cond = model.nozzle, model.gas, model.conditions

    print("\nGeometry and gas properties")
    check("area ratio", noz.area_ratio, 2.402, 1e-3)
    check("contraction ratio", noz.contraction_ratio, 4.00, 1e-3)
    check("throat area", noz.a_throat * 1e6, 3.173, 1e-3, "mm^2")
    check("exit area", noz.a_exit * 1e6, 7.622, 2e-3, "mm^2")
    check("c*", gas.c_star, 430.67, 0.01, "m/s")
    check("exit Mach", exit_mach(model), 2.400, 1e-3)
    check("divergence efficiency", noz.lambda_divergence, 0.9830, 1e-4)
    check("supply pressure", cond.p_supply, 1.480e6, 500.0, "Pa")

    print("\nSteady state at full supply pressure (hand-analysis point)")
    sp = steady_thrust_reference(model)
    me = exit_mach(model)
    t_exit = gas.t_stagnation / (1.0 + 0.2 * me ** 2)
    check("mass flow", sp.mdot * 1000.0, 10.688, 2e-3, "g/s")
    check("exit velocity", sp.exit_velocity, 570.8, 0.1, "m/s")
    check("exit pressure", sp.exit_pressure / 1000.0, 101.3, 0.1, "kPa")
    check("exit temperature", t_exit, 136.2, 0.1, "K")
    check("thrust", sp.thrust, 6.058, 2e-3, "N")
    check("Isp", sp.thrust / (sp.mdot * G0), 57.80, 0.02, "s")
    check_true("throat choked", sp.choked)

    print("\nChoke criterion (trap 1: not the orifice critical ratio)")
    p_choke = first_choke_pressure(model)
    check("first choke / pa", p_choke / cond.p_ambient, 1.0445, 2e-3)
    check_true(
        "well below orifice ratio",
        p_choke / cond.p_ambient < 1.2,
        "a C-D nozzle chokes far below the 1.893 orifice ratio",
    )

    # Mass flow must be continuous through the transition. A wrong criterion
    # produces a ~2.4x jump that shows as a kink in every pulse decay.
    below = steady_state(model, p_choke * 0.999).mdot
    above = steady_state(model, p_choke * 1.001).mdot
    ratio = max(below, above) / max(min(below, above), 1e-12)
    check_true(
        "mdot continuous at choke",
        ratio < 1.10,
        "step = %.1f%% (must stay near the Cd discontinuity, not 2.4x)" % (100 * (ratio - 1)),
    )

    print("\nMonotonicity (thrust must rise with plenum pressure)")
    pressures = [cond.p_ambient * r for r in (1.02, 1.1, 1.5, 3.0, 7.0, 14.0)]
    thrusts = [steady_state(model, p).thrust for p in pressures]
    check_true(
        "thrust monotonic in p0",
        all(b >= a - 1e-9 for a, b in zip(thrusts, thrusts[1:])),
        " -> ".join("%.3f" % t for t in thrusts),
    )

    print("\nCd convention (trap 2)")
    doc = steady_thrust_reference(Model(apply_cd_to_momentum=False)).thrust
    consistent = steady_thrust_reference(Model(apply_cd_to_momentum=True)).thrust
    check("document convention", doc, 6.058, 2e-3, "N")
    check_true(
        "self-consistent is ~2% lower",
        0.015 < (doc - consistent) / doc < 0.025,
        "%.3f N vs %.3f N" % (doc, consistent),
    )

    print("\nTransient sanity")
    result = simulate(
        model,
        Schedule([Phase("open", 0.5, label="Open"), Phase("closed", 0.5, label="Closed")]),
        sample_dt=500e-6,
    )
    settled = result.thrust[result.time < 0.5].max()
    check("settled thrust (with valve drop)", settled, 5.867, 0.05, "N")
    check_true(
        "settled below no-drop reference",
        settled < sp.thrust,
        "%.3f N < %.3f N, the valve costs real pressure" % (settled, sp.thrust)
    )
    check_true(
        "plenum never exceeds supply",
        result.plenum_pressure.max() <= cond.p_supply * (1.0 + 1e-9),
    )
    check_true("thrust never negative", result.thrust.min() >= 0.0)
    check_true(
        "decays to zero after close",
        result.thrust[-1] < 1e-3,
        "final %.2e N" % result.thrust[-1],
    )

    print("\nZero-pressure and edge cases")
    check("thrust at ambient", steady_state(model, cond.p_ambient).thrust, 0.0, 1e-12, "N")
    check_true(
        "no thrust below ambient",
        steady_state(model, cond.p_ambient * 0.5).thrust == 0.0,
    )
    quiet = simulate(model, Schedule([Phase("closed", 0.05)]), sample_dt=5e-3)
    check_true("closed schedule gives no impulse", abs(quiet.total_impulse) < 1e-12)

    print("\nPulse schedule consumes less than steady")
    pulsed = simulate(
        model,
        Schedule([Phase("pulse", 1.0, frequency=20.0, duty_cycle=0.5)]),
        sample_dt=200e-6,
    )
    steady_1s = simulate(
        model, Schedule([Phase("open", 1.0)]), sample_dt=200e-6
    )
    check_true(
        "pulsed impulse < steady impulse",
        pulsed.total_impulse < steady_1s.total_impulse,
        "%.3f < %.3f N*s" % (pulsed.total_impulse, steady_1s.total_impulse),
    )

    if _failures:
        print("\n%d CHECK(S) FAILED: %s\n" % (len(_failures), ", ".join(_failures)))
        return 1
    print("\nAll checks passed.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
