#!/usr/bin/env python3
"""Control panel for the hopper sims: launch runs, compare them, demo them.

    uv run python scripts/gui.py
    uv run python scripts/gui.py --demo        # big fonts, less clutter
    uv run python scripts/gui.py --selftest    # headless checks

Three tabs:

    Run       a front end for scripts/scenario.py - pick a scene and a
              control law, set the parameters, watch the phases go by, read
              the metrics, open the plot.
    Compare   two past runs from outputs/ side by side, metrics diffed and
              both figures shown. This is what the scenario runner was built
              for and had no front end for.
    Live      the native MuJoCo viewer flying under your control, with
              sliders for target altitude and propellant.

WHY THE VIEWER IS A SEPARATE PROCESS. It has to be: MuJoCo's GLFW viewer and
tkinter cannot share one process without corrupting the heap. Measured -
each is clean alone, together they die with "malloc(): unsorted double
linked list corrupted". scripts/simlink.py carries the details and the wire
protocol; scripts/sim_worker.py is the other process.

TKINTER IS NOT THREAD-SAFE. Every subprocess read happens on a reader
thread that only ever puts lines on a queue.Queue; widgets are touched
exclusively from the main loop, which drains that queue via root.after.
"""
import argparse
import json
import pathlib
import queue
import subprocess
import sys
import threading

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

CONTROLLER_MODULE = "example_hover"   # where control laws are looked up

# The gains given entry boxes, grouped by which law reads them. Not every
# settable gain - just the ones worth reaching for while testing a vertical
# law. Anything else goes through --gains on the command line; set_gains() is
# the full list.
#
# This grouping is a LABEL ONLY. Which gains a law actually reads is derived
# from the law's own code by gains_used_by(), and that is what greys a box
# out - so a law you write later gets the right boxes with no edit here, and
# a stale group name can never disable a gain that is really in use.
GAIN_GROUPS = (
    ("hover / hover_pid  (PD core)", ("KP_ALT", "KD_ALT")),
    ("hover_pid  (integral)",        ("KI_ALT", "I_LIMIT_N")),
    ("hover_cascade",                ("KP_POS_V", "KP_V", "V_MAX_MPS")),
    ("2-rate cascade",               ("OUTER_HZ",)),
    ("hover_valve  (thrust tracker)", ("K_VALVE", "GROUND_VZ_MPS")),
)
GAIN_FIELDS = tuple(n for _, names in GAIN_GROUPS for n in names)

# Fields that exist only because a PropellantTank is flying. With
# --fixed-mass the scene's own body mass is used and no tank is built, so
# scenario.py and sim_worker.py never read these. The GUI greys them out and
# the cmd builders below DROP them - they used to be passed and silently
# ignored, which also put meaningless numbers in run.json.
MASS_FIELDS = ("propellant", "dry_mass", "isp")

# Fields that only mean anything with the valve layer on. Like MASS_FIELDS
# these are greyed AND dropped when off, so they cannot put a curve name in
# run.json for a run that never had a valve.
VALVE_FIELDS = ("valve_curve", "valve_feed", "valve_tank_l", "valve_tau")

# Which laws command a 0..1 valve opening rather than Newtons. Read from the
# controller module so it cannot drift from what scenario.py enforces -
# flying a Newtons law through the valve reads 19.44 as "wide open".
def valve_laws():
    try:
        import importlib
        return tuple(getattr(importlib.import_module(CONTROLLER_MODULE),
                             "VALVE_LAWS", ("hover_valve",)))
    except Exception:
        return ("hover_valve",)


# The curve family the rig can hide behind the valve, discovered from
# valve.py rather than listed here, so a new curve appears with no edit.
def valve_curves():
    try:
        import importlib
        return tuple(getattr(importlib.import_module("valve"),
                             "CURVE_NAMES", ("ball",)))
    except Exception:
        return ("ball",)
POLL_MS = 50                          # queue drain / telemetry cadence


# =======================================================================
# DISCOVERY - no hand-maintained lists
# =======================================================================
def find_scenes(scenes_dir=None):
    """Every scene in scenes/, so a new .xml shows up with no code change."""
    d = pathlib.Path(scenes_dir) if scenes_dir else (ROOT / "scenes")
    return sorted(p.name for p in d.glob("*.xml"))


def find_controllers(module_name=CONTROLLER_MODULE):
    """Control laws in a module: functions whose only parameter is `state`.

    Same rule the laws themselves follow, so helpers like set_target and
    world_to_body are skipped without needing a deny-list.
    """
    import importlib
    import inspect
    mod = importlib.import_module(module_name)
    out = []
    for name, fn in vars(mod).items():
        if name.startswith("_") or not callable(fn):
            continue
        try:
            params = inspect.signature(fn).parameters
        except (TypeError, ValueError):
            continue
        if list(params) == ["state"]:
            out.append(name)
    return sorted(out)


def find_runs(outdir=None):
    """Past runs, newest first, each with whatever run.json says about it."""
    d = pathlib.Path(outdir) if outdir else (ROOT / "outputs")
    if not d.is_dir():
        return []
    runs = []
    for p in sorted(d.iterdir(), key=lambda q: q.stat().st_mtime, reverse=True):
        if not p.is_dir():
            continue
        meta = {}
        jf = p / "run.json"
        if jf.is_file():
            try:
                meta = json.loads(jf.read_text())
            except (json.JSONDecodeError, OSError):
                meta = {}
        runs.append({"path": p, "name": p.name, "meta": meta})
    return runs


# gains_used_by lives in scenario.py so the plotter can caption a figure
# with the gains that actually applied. Imported here rather than
# duplicated: two copies of "which gains does this law read" would be two
# answers the moment a law changed. gui.py already imports scenario.
from scenario import gains_used_by          # noqa: E402  (re-exported)


def run_label(run):
    """One readable line for a run, for the Compare list."""
    m = run["meta"]
    prof = m.get("profile", {})
    prop = m.get("propellant")
    bits = [m.get("scene", "?"), m.get("controller", "?")]
    if prof.get("target_alt_m") is not None:
        bits.append(f"target {prof['target_alt_m']:g} m")
    bits.append("fixed mass" if prop is None
                else f"prop {prop.get('propellant_initial_kg', 0):g} kg")
    if m.get("metrics", {}).get("timeouts"):
        bits.append("TIMED OUT")
    return "  |  ".join(bits)


# =======================================================================
# COMMAND BUILDING - pure, so it is testable without a window
# =======================================================================
def build_scenario_cmd(params, python=None, script=None):
    """Turn widget values into a scenario.py argv.

    Only non-empty fields become flags, so blanks inherit scenario.py's own
    defaults instead of this GUI duplicating them.
    """
    py = python or sys.executable
    sc = script or str(HERE / "scenario.py")
    fixed = bool(params.get("fixed_mass"))
    cmd = [py, sc, "--controller",
           f"{CONTROLLER_MODULE}:{params['controller']}",
           "--scene", f"scenes/{params['scene']}"]
    for flag, key in (("--target", "target"),
                      ("--descent-rate", "descent_rate"),
                      ("--control-hz", "control_hz"),
                      ("--propellant", "propellant"),
                      ("--dry-mass", "dry_mass"),
                      ("--isp", "isp"),
                      ("--gravity", "gravity"),
                      ("--gains", "gains"),
                      ("--camera", "camera"),
                      ("--valve-curve", "valve_curve"),
                      ("--valve-feed", "valve_feed"),
                      ("--valve-tank-l", "valve_tank_l"),
                      ("--valve-tau", "valve_tau"),
                      ("--label", "label")):
        if fixed and key in MASS_FIELDS:
            continue      # no tank is built, so these would be ignored
        if key in VALVE_FIELDS and not params.get("valve"):
            continue      # no valve is built, so these would be ignored
        v = params.get(key)
        if v not in (None, ""):
            cmd += [flag, str(v)]
    if fixed:
        cmd.append("--fixed-mass")
    if params.get("valve"):
        cmd.append("--valve")
    if params.get("no_plot"):
        cmd.append("--no-plot")
    if params.get("video"):
        cmd.append("--video")
    if params.get("all_thrusters"):
        cmd.append("--all-thrusters")
    return cmd


def build_worker_cmd(params, port, python=None, script=None):
    """argv for sim_worker.py, the live-viewer process."""
    py = python or sys.executable
    sc = script or str(HERE / "sim_worker.py")
    fixed = bool(params.get("fixed_mass"))
    cmd = [py, sc, "--port", str(port),
           "--controller", f"{CONTROLLER_MODULE}:{params['controller']}",
           "--scene", f"scenes/{params['scene']}"]
    for flag, key in (("--target", "target"),
                      ("--propellant", "propellant"),
                      ("--dry-mass", "dry_mass"),
                      ("--isp", "isp"),
                      ("--gravity", "gravity"),
                      ("--gains", "gains")):
        if fixed and key in MASS_FIELDS:
            continue      # no tank is built, so these would be ignored
        v = params.get(key)
        if v not in (None, ""):
            cmd += [flag, str(v)]
    if fixed:
        cmd.append("--fixed-mass")
    return cmd


def scene_capability(scene_name, override_mass_kg=None, scenes_dir=None):
    """Can this craft lift itself? Compiles the scene and does the sum.

    Returns {mass, weight, thrust_max, twr, can_lift, xml_mass}.

    `override_mass_kg` is the wet mass a PropellantTank would impose. That
    matters because the tank REPLACES the scene's mass rather than adding to
    it: pairing 08_thrust_1D_smol (1 kg craft, 10 N engine) with the 12 kg
    tank defaults needs 19.44 N to hover, so the engine saturates and the
    craft never leaves the ground. Pass None for the scene's own mass.

    Rendered headless with osmesa; no window and no viewer involved.
    """
    import os
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    import mujoco
    d = pathlib.Path(scenes_dir) if scenes_dir else (ROOT / "scenes")
    model = mujoco.MjModel.from_xml_path(str(d / scene_name))
    from thrusters import ThrusterSet
    bid = ThrusterSet(model).body_id()
    xml_mass = float(model.body_mass[bid])
    mass = xml_mass if override_mass_kg is None else float(override_mass_kg)
    weight = mass * abs(float(model.opt.gravity[2]))
    thrust_max = float(model.actuator_ctrlrange[0][1])
    return {"mass": mass, "weight": weight, "thrust_max": thrust_max,
            "twr": (thrust_max / weight) if weight > 0 else float("inf"),
            "can_lift": thrust_max > weight, "xml_mass": xml_mass}


METRIC_ORDER = [
    "peak_altitude_m", "overshoot_m", "ascend_duration_s", "hold_mean_alt_m",
    "hold_max_err_m", "hold_rms_err_m", "peak_descent_rate_mps",
    "commanded_descent_rate_mps", "descent_lag_mean_m", "final_altitude_m",
    "ground_height_m", "return_err_m", "ground_hold_max_err_m",
    "rms_tracking_err_m", "saturated_samples", "saturated_s",
    "mass_initial_kg", "mass_final_kg", "propellant_used_kg",
    "propellant_remaining_kg", "tank_emptied_at_s",
    "mass_bookkeeping_err_kg", "timeouts", "total_duration_s",
]


def diff_metrics(a, b, rel_tol=1e-6):
    """[(key, a_value, b_value, differs)] over the union of both runs' keys.

    Ordered by METRIC_ORDER first so the table reads consistently, then any
    extra keys alphabetically - a run from an older build simply has fewer.
    """
    keys = [k for k in METRIC_ORDER if k in a or k in b]
    keys += sorted((set(a) | set(b)) - set(keys))
    rows = []
    for k in keys:
        va, vb = a.get(k), b.get(k)
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)) \
                and not isinstance(va, bool) and not isinstance(vb, bool):
            scale = max(abs(va), abs(vb), 1e-12)
            differs = abs(va - vb) / scale > rel_tol
        else:
            differs = va != vb
        rows.append((k, va, vb, differs))
    return rows


def fmt_metric(v):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, list):
        return ", ".join(str(x) for x in v) if v else "none"
    return str(v)


# =======================================================================
# THE WINDOW
# =======================================================================
class App:
    def __init__(self, root, demo=False):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.root = root
        self.demo = demo
        root.title("16.83 hopper sim")

        self.base = 13 if demo else 10
        self.mono = ("TkFixedFont", self.base)
        root.option_add("*Font", ("TkDefaultFont", self.base))

        self.proc = None          # scenario.py subprocess
        self.lines = queue.Queue()
        self.last_run = None

        self.worker = None        # sim_worker.py subprocess
        self.worker_srv = None
        self.worker_conn = None
        self.worker_link = None

        self.nb = ttk.Notebook(root)
        self.nb.pack(fill="both", expand=True)
        self.tab_run = ttk.Frame(self.nb)
        self.tab_cmp = ttk.Frame(self.nb)
        self.tab_live = ttk.Frame(self.nb)
        self.nb.add(self.tab_run, text="Run")
        self.nb.add(self.tab_cmp, text="Compare")
        self.nb.add(self.tab_live, text="Live")

        self.scenes = find_scenes()
        self.controllers = find_controllers()
        self._build_run()
        self._build_compare()
        self._build_live()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(POLL_MS, self._pump)

    # ---- shared helpers -------------------------------------------------
    def _defaults(self):
        import scenario
        import propellant
        return {"target": scenario.TARGET_ALT_M,
                "descent_rate": scenario.DESCENT_RATE_MPS,
                "propellant": propellant.PROPELLANT_KG,
                "dry_mass": propellant.DRY_MASS_KG,
                "isp": propellant.ISP_S}

    def params(self):
        """Current widget state as the dict the cmd builders expect."""
        return {"scene": self.v_scene.get(),
                "controller": self.v_ctrl.get(),
                "target": self.v_target.get().strip(),
                "descent_rate": self.v_descent.get().strip(),
                "control_hz": self.v_hz.get().strip(),
                "propellant": self.v_prop.get().strip(),
                "dry_mass": self.v_dry.get().strip(),
                "isp": self.v_isp.get().strip(),
                "gravity": self.v_grav.get().strip(),
                "label": self.v_label.get().strip(),
                "gains": self.gain_spec(self.g_vars,
                                        self._used_gains(self.v_ctrl.get())),
                "fixed_mass": bool(self.v_fixed.get()),
                "valve": bool(self.v_valve.get()),
                "valve_curve": self.v_vcurve.get().strip(),
                "valve_feed": self.v_vfeed.get().strip(),
                "valve_tank_l": self.v_vtank.get().strip(),
                "valve_tau": self.v_vtau.get().strip(),
                "video": bool(self.v_video.get()),
                "all_thrusters": bool(self.v_allthr.get()),
                "no_plot": False}

    @staticmethod
    def gain_spec(vars_map, used=None):
        """Non-empty gain boxes -> 'NAME=VALUE,NAME=VALUE'.

        Blank boxes are omitted so the controller keeps its own default,
        rather than this GUI having to mirror every constant.

        `used` (from gains_used_by) drops gains the selected law does not
        read. Passing them would be harmless to the flight but would record
        a gain in run.json that had no effect on the run - which is exactly
        the kind of thing that makes a sweep's results unreadable later.
        """
        return ",".join(f"{name}={var.get().strip()}"
                        for name, var in vars_map.items()
                        if var.get().strip()
                        and (used is None or name in used))

    # ---- greying out what is not in play --------------------------------
    @staticmethod
    def _set_enabled(widgets, on):
        """Enable/disable (label, entry) pairs, greying the label too.

        ttk disables an Entry via state, but a ttk.Label has no disabled
        look of its own, so the foreground is set explicitly - otherwise a
        dead box still reads as live.
        """
        for lbl, ent in widgets:
            try:
                ent.configure(state="normal" if on else "disabled")
                lbl.configure(foreground="" if on else "#909090")
            except Exception:
                pass

    def _update_gain_fields(self):
        """Grey the gain boxes the selected control law does not read.

        Derived from the law's own code (gains_used_by), not from
        GAIN_GROUPS' labels, so this stays correct for laws added later.
        """
        for prefix, var, widgets, hint in (
                ("", self.v_ctrl, getattr(self, "g_widgets", None),
                 getattr(self, "v_gainhint", None)),
                ("live", getattr(self, "lv_ctrl", None),
                 getattr(self, "lv_g_widgets", None),
                 getattr(self, "lv_gainhint", None))):
            if var is None or not widgets:
                continue
            law = var.get()
            try:
                used = gains_used_by(law)
            except Exception:
                used = set(GAIN_FIELDS)        # fail open, never grey wrongly
            for name, pair in widgets.items():
                self._set_enabled([pair], name in used)
            if hint is not None:
                live = [n for n in GAIN_FIELDS if n in used]
                hint.set(f"{law} reads: " + (", ".join(live) if live
                                             else "none of these"))

    def _update_valve_fields(self):
        """Grey the valve boxes when no valve is flying, and the checkbox
        itself when the selected law cannot command one.

        Two separate reasons, both worth showing rather than discovering:

          the LAW       only laws in example_hover.VALVE_LAWS return a 0..1
                        opening. Flying one that returns Newtons through the
                        valve reads 19.44 as "wide open, clamped" and the
                        engine runs flat out for the whole flight - which
                        looks like a wild controller, not a units mismatch.
                        scenario.py refuses it; the GUI should not offer it.
          the CHECKBOX  with the valve off, curve / feed / bottle / lag are
                        never read, so they are greyed AND dropped from the
                        argv - the same treatment MASS_FIELDS gets under
                        fixed mass, and for the same reason: a flag that is
                        passed and ignored still lands in run.json.
        """
        if not hasattr(self, "cb_valve"):
            return
        law = self.v_ctrl.get()
        laws = valve_laws()
        can = law in laws
        try:
            self.cb_valve.configure(state="normal" if can else "disabled")
        except Exception:
            pass
        if not can and self.v_valve.get():
            self.v_valve.set(0)       # cannot silently fly an invalid combo
        on = can and bool(self.v_valve.get())
        self._set_enabled(self.valve_widgets, on)
        if hasattr(self, "v_valvehint"):
            if not can:
                self.v_valvehint.set(
                    f"{law} commands Newtons directly.\n"
                    f"valve laws: {', '.join(laws)}")
            elif on:
                self.v_valvehint.set(
                    "the law sets a 0..1 opening;\n"
                    "an unknown curve makes the force")
            else:
                self.v_valvehint.set("")

    def _update_mass_fields(self):
        """Grey propellant / dry mass / Isp when flying fixed mass.

        With --fixed-mass no PropellantTank is built, so the runner never
        reads any of the three. They were previously still editable AND
        still passed on the command line, where they were silently ignored.
        """
        if hasattr(self, "mass_widgets"):
            self._set_enabled(self.mass_widgets, not bool(self.v_fixed.get()))
        if hasattr(self, "lv_mass_widgets"):
            variable = self.lv_massmode.get() != "fixed"
            self._set_enabled(self.lv_mass_widgets, variable)
            for w in getattr(self, "lv_mass_buttons", []):
                try:
                    w.configure(state="normal" if variable else "disabled")
                except Exception:
                    pass
            # The propellant SLIDER is live-only and equally meaningless
            # under fixed mass: sim_worker ignores the command because
            # there is no tank to refill.
            if hasattr(self, "s_prop"):
                try:
                    self.s_prop.configure(state="normal" if variable
                                          else "disabled")
                    self.lbl_prop.configure(foreground="" if variable
                                            else "#909090")
                except Exception:
                    pass

    # ---- Run tab --------------------------------------------------------
    def _build_run(self):
        tk, ttk = self.tk, self.ttk
        d = self._defaults()
        top = ttk.Frame(self.tab_run, padding=8)
        top.pack(fill="x")

        self.v_scene = tk.StringVar(value="07_thrust_1D.xml"
                                    if "07_thrust_1D.xml" in self.scenes
                                    else (self.scenes[0] if self.scenes else ""))
        self.v_ctrl = tk.StringVar(value="hover" if "hover" in self.controllers
                                   else (self.controllers[0] if self.controllers else ""))
        ttk.Label(top, text="Scene").grid(row=0, column=0, sticky="w")
        ttk.Combobox(top, textvariable=self.v_scene, values=self.scenes,
                     state="readonly", width=26).grid(row=0, column=1, sticky="w")
        ttk.Label(top, text="Controller").grid(row=0, column=2, sticky="e", padx=(12, 4))
        ttk.Combobox(top, textvariable=self.v_ctrl, values=self.controllers,
                     state="readonly", width=20).grid(row=0, column=3, sticky="w")

        self.grid_params = ttk.Frame(self.tab_run, padding=(8, 0))
        self.grid_params.pack(fill="x")
        # Returns the (label, entry) pair so the mass-only fields can be
        # greyed out when --fixed-mass makes them meaningless.
        def field(col, label, var, width=9):
            f = ttk.Frame(self.grid_params)
            f.grid(row=0, column=col, sticky="w", padx=(0, 10))
            lbl = ttk.Label(f, text=label)
            lbl.pack(anchor="w")
            ent = ttk.Entry(f, textvariable=var, width=width)
            ent.pack()
            return lbl, ent
        self.v_target = tk.StringVar(value=str(d["target"]))
        self.v_descent = tk.StringVar(value=str(d["descent_rate"]))
        self.v_hz = tk.StringVar(value="")
        self.v_prop = tk.StringVar(value=str(d["propellant"]))
        self.v_dry = tk.StringVar(value=str(d["dry_mass"]))
        self.v_isp = tk.StringVar(value=str(d["isp"]))
        self.v_grav = tk.StringVar(value="")
        self.v_label = tk.StringVar(value="")
        self.v_fixed = tk.IntVar(value=0)
        field(0, "target [m]", self.v_target)
        field(1, "descent [m/s]", self.v_descent)
        field(2, "control [Hz]", self.v_hz)
        # Collected so _update_mass_fields can disable them: with fixed mass
        # no PropellantTank is built and these three are never read.
        self.mass_widgets = [field(3, "propellant [kg]", self.v_prop),
                             field(4, "dry mass [kg]", self.v_dry),
                             field(5, "Isp [s]", self.v_isp)]
        field(6, "gravity", self.v_grav)
        field(7, "label", self.v_label, 12)
        ttk.Checkbutton(self.grid_params, text="fixed mass",
                        variable=self.v_fixed,
                        command=self._update_mass_fields
                        ).grid(row=0, column=8, sticky="w")
        # Recording costs about 10 s of wall time on an 18 s profile and
        # cannot change the flight (scenario.py's selftest asserts the CSV
        # is byte-identical), so it is a plain opt-in checkbox.
        self.v_video = tk.IntVar(value=0)
        ttk.Checkbutton(self.grid_params, text="record mp4",
                        variable=self.v_video
                        ).grid(row=0, column=9, sticky="w", padx=(10, 0))
        # Off by default: on a vertical law the four RCS nozzles are exactly
        # 0.0 N throughout, and plotting them drags the thrust axis down to
        # include zero, squeezing the main engine's trace into a fifth of
        # the panel. The CSV keeps every column either way.
        self.v_allthr = tk.IntVar(value=0)
        ttk.Checkbutton(self.grid_params, text="plot idle thrusters",
                        variable=self.v_allthr
                        ).grid(row=0, column=10, sticky="w", padx=(10, 0))

        # Gains, grouped by the law that reads them, in a labelled box per
        # group - so it is visible at a glance that KI_ALT does nothing to
        # the cascade and KP_V does nothing to the PID. On top of the
        # grouping, boxes the SELECTED law does not read are disabled, which
        # is derived from the law's code rather than from these labels.
        gbox = ttk.Frame(self.tab_run, padding=(8, 4))
        gbox.pack(fill="x")
        self.g_vars = {}
        self.g_widgets = {}
        for gcol, (title, names) in enumerate(GAIN_GROUPS):
            grp = ttk.Labelframe(gbox, text=title, padding=(6, 2, 6, 4))
            grp.grid(row=0, column=gcol, sticky="nw", padx=(0, 10))
            for col, name in enumerate(names):
                lbl = ttk.Label(grp, text=name)
                lbl.grid(row=0, column=col, sticky="w", padx=(0, 6))
                var = tk.StringVar(value="")
                ent = ttk.Entry(grp, textvariable=var, width=9)
                ent.grid(row=1, column=col, sticky="w", padx=(0, 6))
                self.g_vars[name] = var
                self.g_widgets[name] = (lbl, ent)
        # The valve layer, in its own labelled box because it changes what
        # the controller's numbers MEAN: with it on, the main engine is
        # commanded as a 0..1 opening and an unknown curve turns that into
        # Newtons. Only laws that return an opening can be flown this way,
        # and the checkbox disables itself for the others rather than
        # letting a run fail in the driver.
        vbox = ttk.Labelframe(self.tab_run, text="valve layer  (the "
                              "controller is NOT told the thrust curve)",
                              padding=(8, 2, 8, 4))
        vbox.pack(fill="x", padx=8, pady=(2, 0))
        self.v_valve = tk.IntVar(value=0)
        self.cb_valve = ttk.Checkbutton(
            vbox, text="command a valve", variable=self.v_valve,
            command=self._update_valve_fields)
        self.cb_valve.grid(row=0, column=0, rowspan=2, sticky="w", padx=(0, 12))
        self.v_vcurve = tk.StringVar(value="")
        self.v_vfeed = tk.StringVar(value="")
        self.v_vtank = tk.StringVar(value="")
        self.v_vtau = tk.StringVar(value="")

        def vfield(col, text, var, values=None, width=10):
            lbl = ttk.Label(vbox, text=text)
            lbl.grid(row=0, column=col, sticky="w", padx=(0, 8))
            if values is None:
                w = ttk.Entry(vbox, textvariable=var, width=width)
            else:
                w = ttk.Combobox(vbox, textvariable=var, values=values,
                                 state="readonly", width=width)
            w.grid(row=1, column=col, sticky="w", padx=(0, 8))
            return lbl, w

        # Curves discovered from valve.py, so a new one needs no edit here.
        # "" means "use valve.py's own default", like every other blank box.
        self.valve_widgets = [
            vfield(1, "curve", self.v_vcurve, ("",) + valve_curves(), 11),
            vfield(2, "feed", self.v_vfeed,
                   ("", "constant", "regulated", "from_tank"), 11),
            vfield(3, "bottle [L]", self.v_vtank),
            vfield(4, "lag tau [s]", self.v_vtau),
        ]
        self.v_valvehint = tk.StringVar(value="")
        ttk.Label(vbox, textvariable=self.v_valvehint, foreground="#606060"
                  ).grid(row=0, column=5, rowspan=2, sticky="w", padx=(6, 0))

        self.v_gainhint = tk.StringVar(value="")
        ttk.Label(gbox, textvariable=self.v_gainhint, foreground="#606060"
                  ).grid(row=0, column=len(GAIN_GROUPS), sticky="nw",
                         padx=(4, 0))
        self.v_ctrl.trace_add("write", lambda *_: self._update_gain_fields())
        self.v_ctrl.trace_add("write", lambda *_: self._update_valve_fields())
        self._update_gain_fields()
        self._update_mass_fields()
        self._update_valve_fields()

        bar = ttk.Frame(self.tab_run, padding=8)
        bar.pack(fill="x")
        self.b_run = ttk.Button(bar, text="Run scenario", command=self.start_run)
        self.b_run.pack(side="left")
        self.b_stop = ttk.Button(bar, text="Stop", command=self.stop_run,
                                 state="disabled")
        self.b_stop.pack(side="left", padx=6)
        ttk.Button(bar, text="Open plot", command=self.open_plot).pack(side="left")
        self.v_status = tk.StringVar(value="idle")
        ttk.Label(bar, textvariable=self.v_status).pack(side="left", padx=14)

        self.log = tk.Text(self.tab_run, height=18, font=self.mono, wrap="none")
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        if self.demo:
            self.grid_params.pack_forget()
            self.log.configure(height=8)

    def busy(self):
        """Is a scenario run still in flight?

        Checks poll() as well as the handle, because the child can exit a
        moment before _pump drains the reader thread's sentinel and clears
        self.proc. Testing the handle alone leaves a window - up to POLL_MS -
        in which a run looks active although it has finished.
        """
        return self.proc is not None and self.proc.poll() is None

    def start_run(self):
        # Silently ignoring the click is how a dropped run goes unnoticed:
        # say so instead.
        if self.busy():
            self.v_status.set("a run is already in progress")
            return
        if self.proc is not None:
            # Finished but not yet reaped by _pump; reap it now so this
            # click is not lost.
            self._run_finished()
        cmd = build_scenario_cmd(self.params())
        self.log.delete("1.0", "end")
        self.log.insert("end", " ".join(cmd) + "\n\n")
        self.v_status.set("running...")
        self.b_run.configure(state="disabled")
        self.b_stop.configure(state="normal")
        self.proc = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     bufsize=1)

        def reader(proc, q):
            # Reader THREAD: only ever touches the queue, never a widget.
            for line in proc.stdout:
                q.put(line)
            proc.stdout.close()
            q.put(None)
        threading.Thread(target=reader, args=(self.proc, self.lines),
                         daemon=True).start()

    def stop_run(self):
        if self.proc is None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.v_status.set("stopped")

    def _run_finished(self):
        if self.proc is None:
            return                     # already reaped
        rc = self.proc.poll()
        self.proc = None
        self.b_run.configure(state="normal")
        self.b_stop.configure(state="disabled")
        runs = find_runs()
        self.last_run = runs[0] if runs else None
        if self.last_run and rc == 0:
            m = self.last_run["meta"].get("metrics", {})
            self.v_status.set(f"done: peak {m.get('peak_altitude_m', 0):.4f} m, "
                              f"overshoot {m.get('overshoot_m', 0):.4f} m")
            self.log.insert("end", "\n--- metrics ---\n")
            for k, v in m.items():
                self.log.insert("end", f"  {k:<30s} {fmt_metric(v)}\n")
            self.log.see("end")
        else:
            self.v_status.set(f"exited {rc}")
        self.refresh_runs()

    def open_plot(self):
        run = self.last_run or (find_runs()[0] if find_runs() else None)
        if run is None:
            self.v_status.set("no runs yet")
            return
        # Interactive first, the static PNG as the fallback. The figure is
        # REBUILT from flight.csv + run.json rather than read off the PNG,
        # because scenario.py runs as a subprocess and its figure object
        # died with it - see scenario.load_run.
        try:
            self._show_figure_window(run["path"], run["name"])
            return
        except Exception as e:
            self.v_status.set(f"interactive plot unavailable: "
                              f"{type(e).__name__}")
        png = run["path"] / "flight.png"
        if not png.is_file():
            self.v_status.set("that run has no flight.png")
            return
        self._show_png_window(png, run["name"])

    @staticmethod
    def _select_tk_backend():
        """Put matplotlib on TkAgg before any figure is built.

        scenario.plot_flight only forces Agg when the current backend is
        still Agg or template, so this has to run FIRST - otherwise the
        figure is built for a canvas that cannot be drawn in a window.
        Selecting a backend after pyplot is imported is a no-op, which is
        why this is called before scenario is imported here.
        """
        import matplotlib
        if matplotlib.get_backend().lower() not in ("tkagg",):
            matplotlib.use("TkAgg", force=True)
        # Tk draws the canvas before the window has its real geometry, and
        # again from a deferred callback during teardown - at both moments
        # the canvas is near-zero and constrained_layout warns that the axes
        # "collapsed to zero". Measured once the window settles: the panels
        # sit at 0.13-0.18 of figure height from 1200x400 up to 1200x1100,
        # so this is transient noise, not a sizing problem. Filtered here
        # rather than around one draw() because the deferred draws happen
        # inside tkinter's dispatcher, out of reach of any local guard.
        import warnings
        warnings.filterwarnings(
            "ignore", category=UserWarning,
            message=".*constrained_layout not applied.*")

    def _figure_window(self, title, height_frac=0.92):
        """A Toplevel sized from the SCREEN, not from the figure.

        The figure is 11x14 in (5 panels) or 11x16.8 in (6), which at 110 dpi
        is taller than any display. A packed canvas resizes with the window
        and constrained_layout reflows, so the panels just get shorter and
        zoom is how detail is reached - no scrollbar needed.
        """
        win = self.tk.Toplevel(self.root)
        win.title(title)
        h = int(self.root.winfo_screenheight() * height_frac)
        w = int(min(self.root.winfo_screenwidth() * 0.9, h * 1.05))
        win.geometry(f"{w}x{h}")
        return win

    def _embed_figure(self, fig, master):
        """Put a matplotlib figure in a Tk frame with the real toolbar.

        Imported here, not at module scope, so a matplotlib problem cannot
        stop the GUI from starting - open_plot falls back to the PNG.

        A TkAgg canvas is pure Tk with no GL, so it is safe in this process.
        A MuJoCo GLFW viewer is NOT - see the heap-corruption note at the top
        of this file - which is why the interactive figure lives here and not
        in a process that opens a viewer.
        """
        from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                                       NavigationToolbar2Tk)
        frame = self.tk.Frame(master)
        canvas = FigureCanvasTkAgg(fig, master=frame)
        toolbar = NavigationToolbar2Tk(canvas, frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side="top", fill="x")
        canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        canvas.draw()
        self._bind_scroll_zoom(canvas, fig)
        return frame, canvas

    @staticmethod
    def _bind_scroll_zoom(canvas, fig, factor=1.25):
        """Scroll wheel zooms the TIME axis about the cursor.

        The toolbar's box-zoom is awkward on a tall stack of time series -
        you want to narrow the time window, not draw a rectangle in one
        panel. Scrolling scales xlim about the cursor instead, and because
        every panel shares the x axis one handler moves them all.
        """
        def on_scroll(event):
            if event.inaxes is None:
                return
            step = event.step if event.step else (
                1.0 if event.button == "up" else -1.0)
            scale = (1.0 / factor) if step > 0 else factor
            x = event.xdata
            if x is None:
                return
            lo, hi = event.inaxes.get_xlim()
            event.inaxes.set_xlim(x - (x - lo) * scale,
                                  x + (hi - x) * scale)
            canvas.draw_idle()
        canvas.mpl_connect("scroll_event", on_scroll)

    def _show_figure_window(self, run_path, title):
        """The interactive view of one run."""
        self._select_tk_backend()
        import scenario
        fig, _info = scenario.figure_for_run(run_path)
        win = self._figure_window(title)
        frame, _canvas = self._embed_figure(fig, win)
        frame.pack(fill="both", expand=True)
        # Release the figure when the window goes, or every open leaks one.
        def _close():
            try:
                import matplotlib.pyplot as plt
                plt.close(fig)
            except Exception:
                pass
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", _close)
        return win

    def _show_png_window(self, png, title):
        tk = self.tk
        from PIL import Image, ImageTk
        win = tk.Toplevel(self.root)
        win.title(title)
        img = Image.open(png)
        scale = min(1.0, 900 / img.height)
        if scale < 1.0:
            img = img.resize((int(img.width * scale), int(img.height * scale)))
        ph = ImageTk.PhotoImage(img)
        lbl = tk.Label(win, image=ph)
        lbl.image = ph                  # keep a reference or Tk drops it
        lbl.pack()

    # ---- Compare tab ----------------------------------------------------
    def _build_compare(self):
        tk, ttk = self.tk, self.ttk
        bar = ttk.Frame(self.tab_cmp, padding=8)
        bar.pack(fill="x")
        ttk.Label(bar, text="Tick two runs, then compare").pack(side="left")
        ttk.Button(bar, text="Refresh", command=self.refresh_runs).pack(side="left", padx=8)
        ttk.Button(bar, text="Compare", command=self.do_compare).pack(side="left")
        ttk.Button(bar, text="Show both figures",
                   command=self.show_both).pack(side="left", padx=6)
        self.v_cmp_status = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.v_cmp_status).pack(side="left", padx=12)

        self.runlist = tk.Listbox(self.tab_cmp, selectmode="extended",
                                  height=8, font=self.mono)
        self.runlist.pack(fill="x", padx=8)
        self.cmp_out = tk.Text(self.tab_cmp, height=22, font=self.mono, wrap="none")
        self.cmp_out.pack(fill="both", expand=True, padx=8, pady=8)
        self.cmp_out.tag_configure("diff", foreground="#b00000")
        self.cmp_out.tag_configure("hdr", font=("TkFixedFont", self.base, "bold"))
        self._runs_cache = []
        self.refresh_runs()

    def refresh_runs(self):
        self._runs_cache = find_runs()
        if not hasattr(self, "runlist"):
            return
        self.runlist.delete(0, "end")
        for r in self._runs_cache:
            self.runlist.insert("end", run_label(r))

    def _selected_runs(self):
        return [self._runs_cache[i] for i in self.runlist.curselection()]

    def do_compare(self):
        sel = self._selected_runs()
        if len(sel) != 2:
            self.v_cmp_status.set(f"pick exactly 2 (you picked {len(sel)})")
            return
        a, b = sel
        self.v_cmp_status.set("")
        self.cmp_out.delete("1.0", "end")
        self.cmp_out.insert("end", f"A  {a['name']}\nB  {b['name']}\n", "hdr")
        # Name the tuning each run flew. Without this a directory of sweep
        # results is unidentifiable - the metrics differ and you cannot say
        # why.
        for tag, run in (("A", a), ("B", b)):
            ov = run["meta"].get("gains_overridden")
            self.cmp_out.insert(
                "end", f"   {tag} gains: "
                       + (", ".join(f"{k}={v}" for k, v in ov.items())
                          if ov else "(all defaults)") + "\n")
        self.cmp_out.insert("end", "\n")
        self.cmp_out.insert("end", f"{'metric':<30s} {'A':>16s} {'B':>16s}\n", "hdr")
        rows = diff_metrics(a["meta"].get("metrics", {}),
                            b["meta"].get("metrics", {}))
        ndiff = 0
        for k, va, vb, differs in rows:
            line = f"{k:<30s} {fmt_metric(va):>16s} {fmt_metric(vb):>16s}"
            line += "   <-- differs\n" if differs else "\n"
            self.cmp_out.insert("end", line, "diff" if differs else "")
            ndiff += bool(differs)
        self.cmp_out.insert("end", f"\n{ndiff} of {len(rows)} metrics differ\n", "hdr")

    def show_both(self):
        sel = self._selected_runs()
        if len(sel) != 2:
            self.v_cmp_status.set(f"pick exactly 2 (you picked {len(sel)})")
            return
        try:
            self._show_both_interactive(sel)
            return
        except Exception as e:
            self.v_cmp_status.set(f"interactive compare unavailable: "
                                  f"{type(e).__name__}")
        self._show_both_png(sel)

    def _show_both_interactive(self, sel):
        """Two runs side by side, panning and zooming TOGETHER.

        The config panel is dropped here: at half width each it would eat
        the space the traces need, and the Compare tab already prints the
        gains as text. Linking the time axes is the point - comparing two
        tunings means looking at the same instant in both.
        """
        self._select_tk_backend()
        import scenario
        tk = self.tk
        win = self._figure_window("side by side", height_frac=0.9)
        figs = []
        for col, run in enumerate(sel):
            frame = tk.Frame(win)
            frame.grid(row=0, column=col, sticky="nsew")
            win.grid_columnconfigure(col, weight=1)
            tk.Label(frame, text=("A: " if col == 0 else "B: ") + run["name"],
                     font=("TkDefaultFont", self.base)).pack()
            fig, _ = scenario.figure_for_run(run["path"], config=False)
            figs.append(fig)
            inner, _canvas = self._embed_figure(fig, frame)
            inner.pack(fill="both", expand=True)
        win.grid_rowconfigure(0, weight=1)
        # sharex ACROSS figures is allowed, and is what makes the two pan
        # as one. Within a figure the panels already share x, so linking the
        # first axis of each links everything.
        if len(figs) == 2 and figs[0].axes and figs[1].axes:
            figs[0].axes[0].sharex(figs[1].axes[0])

        def _close():
            try:
                import matplotlib.pyplot as plt
                for f in figs:
                    plt.close(f)
            except Exception:
                pass
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", _close)
        return win

    def _show_both_png(self, sel):
        """The original static fallback, kept for when matplotlib cannot."""
        tk = self.tk
        from PIL import Image, ImageTk
        win = tk.Toplevel(self.root)
        win.title("side by side")
        for col, run in enumerate(sel):
            png = run["path"] / "flight.png"
            frame = tk.Frame(win)
            frame.grid(row=0, column=col, padx=4)
            tk.Label(frame, text=("A: " if col == 0 else "B: ") + run["name"],
                     font=("TkDefaultFont", self.base)).pack()
            if not png.is_file():
                tk.Label(frame, text="(no flight.png)").pack()
                continue
            img = Image.open(png)
            scale = min(1.0, 820 / img.height)
            img = img.resize((int(img.width * scale), int(img.height * scale)))
            ph = ImageTk.PhotoImage(img)
            lbl = tk.Label(frame, image=ph)
            lbl.image = ph
            lbl.pack()

    # ---- Live tab -------------------------------------------------------
    def _build_live(self):
        tk, ttk = self.tk, self.ttk

        # The Live tab gets its OWN scene/controller/mass controls. It used
        # to read the Run tab's widgets, so launching here silently depended
        # on what was selected on another tab - and there was no way to tell
        # whether you were flying fixed or variable mass.
        pick = ttk.Frame(self.tab_live, padding=(8, 8, 8, 0))
        pick.pack(fill="x")
        ttk.Label(pick, text="Scene").grid(row=0, column=0, sticky="w")
        self.lv_scene = tk.StringVar(value=self.v_scene.get())
        ttk.Combobox(pick, textvariable=self.lv_scene, values=self.scenes,
                     state="readonly", width=26).grid(row=0, column=1, sticky="w")
        ttk.Label(pick, text="Controller").grid(row=0, column=2, sticky="e",
                                                padx=(12, 4))
        self.lv_ctrl = tk.StringVar(value=self.v_ctrl.get())
        ttk.Combobox(pick, textvariable=self.lv_ctrl, values=self.controllers,
                     state="readonly", width=20).grid(row=0, column=3, sticky="w")

        mass = ttk.Frame(self.tab_live, padding=(8, 6, 8, 0))
        mass.pack(fill="x")
        self.lv_massmode = tk.StringVar(value="variable")
        ttk.Radiobutton(mass, text="variable mass (propellant burns)",
                        variable=self.lv_massmode, value="variable",
                        command=self._on_mass_mode
                        ).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(mass, text="fixed mass (scene's own)",
                        variable=self.lv_massmode, value="fixed",
                        command=self._on_mass_mode
                        ).grid(row=0, column=1, sticky="w", padx=(14, 0))
        # Collected as (label, entry) pairs so fixed mass can grey them:
        # sim_worker builds no tank in that mode and reads none of them.
        l_dry = ttk.Label(mass, text="dry [kg]")
        l_dry.grid(row=0, column=2, sticky="e", padx=(16, 3))
        self.lv_dry = tk.StringVar(value=self.v_dry.get())
        e_dry = ttk.Entry(mass, textvariable=self.lv_dry, width=7)
        e_dry.grid(row=0, column=3)
        l_prop = ttk.Label(mass, text="prop [kg]")
        l_prop.grid(row=0, column=4, sticky="e", padx=(10, 3))
        self.lv_prop = tk.StringVar(value=self.v_prop.get())
        e_prop = ttk.Entry(mass, textvariable=self.lv_prop, width=7)
        e_prop.grid(row=0, column=5)
        b_match = ttk.Button(mass, text="match scene",
                             command=self._match_scene_mass)
        b_match.grid(row=0, column=6, padx=(10, 0))
        self.lv_mass_widgets = [(l_dry, e_dry), (l_prop, e_prop)]
        self.lv_mass_buttons = [b_match]

        # Same grouped layout as the Run tab, same derived greying. Gains
        # apply at LAUNCH here - there is no gain command in the worker
        # protocol - so editing these mid-flight changes nothing until the
        # viewer is relaunched.
        gains = ttk.Frame(self.tab_live, padding=(8, 6, 8, 0))
        gains.pack(fill="x")
        ttk.Label(gains, text="gains at launch (blank = default):").grid(
            row=0, column=0, sticky="nw", padx=(0, 8))
        self.lv_g_vars = {}
        self.lv_g_widgets = {}
        for gcol, (title, names) in enumerate(GAIN_GROUPS):
            grp = ttk.Labelframe(gains, text=title, padding=(6, 2, 6, 4))
            grp.grid(row=0, column=1 + gcol, sticky="nw", padx=(0, 10))
            for col, name in enumerate(names):
                lbl = ttk.Label(grp, text=name)
                lbl.grid(row=0, column=col, sticky="w", padx=(0, 6))
                var = tk.StringVar(value="")
                ent = ttk.Entry(grp, textvariable=var, width=7)
                ent.grid(row=1, column=col, sticky="w", padx=(0, 6))
                self.lv_g_vars[name] = var
                self.lv_g_widgets[name] = (lbl, ent)
        self.lv_gainhint = tk.StringVar(value="")
        ttk.Label(gains, textvariable=self.lv_gainhint, foreground="#606060"
                  ).grid(row=0, column=1 + len(GAIN_GROUPS), sticky="nw")

        self.v_hint = tk.StringVar(value="")
        self.lbl_hint = ttk.Label(self.tab_live, textvariable=self.v_hint,
                                  padding=(8, 4), foreground="#a05000")
        self.lbl_hint.pack(fill="x")
        for var in (self.lv_scene, self.lv_dry, self.lv_prop):
            var.trace_add("write", lambda *_: self._update_mass_hint())

        bar = ttk.Frame(self.tab_live, padding=8)
        bar.pack(fill="x")
        self.b_launch = ttk.Button(bar, text="Launch viewer",
                                   command=self.launch_worker)
        self.b_launch.pack(side="left")
        self.b_kill = ttk.Button(bar, text="Close viewer",
                                 command=self.kill_worker, state="disabled")
        self.b_kill.pack(side="left", padx=6)
        for txt, cmd in (("Pause", "pause"), ("Resume", "resume"),
                         ("Reset", "reset")):
            ttk.Button(bar, text=txt,
                       command=lambda c=cmd: self.send_worker({"cmd": c})
                       ).pack(side="left", padx=2)

        sl = ttk.Frame(self.tab_live, padding=8)
        sl.pack(fill="x")
        # Bound through a DoubleVar with a trace, NOT the Scale's `command`.
        # Scale.set() does not fire `command` - that only runs on a user
        # drag - so a programmatic set (including the one in launch_worker)
        # would silently never reach the worker. A variable trace fires for
        # both, which is what makes the slider scriptable and testable.
        ttk.Label(sl, text="target altitude [m]").grid(row=0, column=0, sticky="w")
        self.var_target = tk.DoubleVar(value=2.0)
        self.s_target = tk.Scale(sl, from_=0.4, to=6.0, resolution=0.05,
                                 orient="horizontal", length=380,
                                 variable=self.var_target)
        self.s_target.grid(row=0, column=1, sticky="w")
        self.var_target.trace_add("write", self._on_target_var)

        self.lbl_prop = ttk.Label(sl, text="propellant [kg]")
        self.lbl_prop.grid(row=1, column=0, sticky="w")
        self.var_prop = tk.DoubleVar(value=4.0)
        self.s_prop = tk.Scale(sl, from_=0.0, to=8.0, resolution=0.05,
                               orient="horizontal", length=380,
                               variable=self.var_prop)
        self.s_prop.grid(row=1, column=1, sticky="w")
        self.var_prop.trace_add("write", self._on_prop_var)

        self.v_mode = tk.StringVar(value="")
        ttk.Label(self.tab_live, textvariable=self.v_mode, padding=(8, 2),
                  font=("TkDefaultFont", self.base, "bold")).pack(anchor="w")
        self.v_tel = tk.StringVar(value="viewer not running")
        ttk.Label(self.tab_live, textvariable=self.v_tel,
                  font=("TkFixedFont", self.base + (6 if self.demo else 2)),
                  padding=10).pack(anchor="w")
        self.live_log = tk.Text(self.tab_live, height=6, font=self.mono)
        self.live_log.pack(fill="both", expand=True, padx=8, pady=8)
        if self.demo:
            self.live_log.pack_forget()

        self.lv_ctrl.trace_add("write", lambda *_: self._update_gain_fields())
        self._update_gain_fields()
        self._update_mass_fields()

    def live_params(self):
        """Live-tab widget state, independent of the Run tab."""
        return {"scene": self.lv_scene.get(),
                "controller": self.lv_ctrl.get(),
                "target": f"{float(self.var_target.get()):g}",
                "propellant": self.lv_prop.get().strip(),
                "dry_mass": self.lv_dry.get().strip(),
                "isp": self.v_isp.get().strip(),
                "gravity": self.v_grav.get().strip(),
                "gains": self.gain_spec(self.lv_g_vars,
                                        self._used_gains(self.lv_ctrl.get())),
                "fixed_mass": self.lv_massmode.get() == "fixed"}

    @staticmethod
    def _used_gains(law):
        """gains_used_by, failing open so a lookup problem never drops a
        gain the user deliberately typed."""
        try:
            return gains_used_by(law)
        except Exception:
            return None

    def _on_mass_mode(self):
        """Mass mode changed: re-grey the tank fields and redo the hint."""
        self._update_mass_fields()
        self._update_mass_hint()

    def _update_mass_hint(self):
        """Say up front whether the craft can lift at all.

        A light scene flown with heavy tank defaults is pinned to the ground
        and the target slider looks dead. Better to say so before launching
        than to let it look like a broken control.
        """
        try:
            info = scene_capability(self.lv_scene.get(),
                                    None if self.lv_massmode.get() == "fixed"
                                    else (float(self.lv_dry.get() or 0)
                                          + float(self.lv_prop.get() or 0)))
        except (ValueError, OSError, KeyError):
            self.v_hint.set("")
            return
        mode = ("fixed mass" if self.lv_massmode.get() == "fixed"
                else "variable mass")
        msg = (f"{mode}:  {info['mass']:.3f} kg   needs {info['weight']:.2f} N "
               f"to hover   engine {info['thrust_max']:.1f} N   "
               f"T/W {info['twr']:.2f}")
        if not info["can_lift"]:
            msg += "   <-- CANNOT LIFT: engine is weaker than the weight"
        self.v_hint.set(msg)
        try:
            self.lbl_hint.configure(
                foreground="#b00000" if not info["can_lift"] else "#206020")
        except Exception:
            pass

    def _match_scene_mass(self):
        """Set dry+propellant to the scene's own mass, 90/10."""
        try:
            info = scene_capability(self.lv_scene.get(), None)
        except (ValueError, OSError, KeyError):
            return
        m = info["mass"]
        self.lv_dry.set(f"{m * 0.9:g}")
        self.lv_prop.set(f"{m * 0.1:g}")
        self._update_mass_hint()

    def _on_target_var(self, *_):
        self.send_worker({"cmd": "target", "value": float(self.var_target.get())})

    def _on_prop_var(self, *_):
        self.send_worker({"cmd": "propellant",
                          "value": float(self.var_prop.get())})

    def launch_worker(self):
        if self.worker is not None:
            return
        from simlink import LineReader, listening_socket
        self.worker_srv, port = listening_socket()
        cmd = build_worker_cmd(self.live_params(), port)
        self.live_log.insert("end", " ".join(cmd) + "\n")
        self.worker = subprocess.Popen(cmd, cwd=str(ROOT),
                                       stdout=subprocess.DEVNULL,
                                       stderr=subprocess.PIPE, text=True)
        self.worker_srv.settimeout(20)
        try:
            self.worker_conn, _ = self.worker_srv.accept()
        except OSError as e:
            self.live_log.insert("end", f"worker never connected: {e}\n")
            self.kill_worker()
            return
        self.worker_conn.setblocking(False)
        self.worker_link = LineReader(self.worker_conn)
        self._update_mass_hint()
        self.b_launch.configure(state="disabled")
        self.b_kill.configure(state="normal")

    def send_worker(self, msg):
        if self.worker_conn is None:
            return
        from simlink import send
        if not send(self.worker_conn, msg):
            self.live_log.insert("end", "link to viewer lost\n")

    def kill_worker(self):
        """Ask nicely, then insist. Never leave an orphan viewer behind."""
        if self.worker is None:
            self._clear_worker()
            return
        self.send_worker({"cmd": "quit"})
        try:
            self.worker.wait(timeout=6)
        except subprocess.TimeoutExpired:
            self.worker.terminate()
            try:
                self.worker.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.worker.kill()
        self._clear_worker()

    def _clear_worker(self):
        for obj in (self.worker_conn, self.worker_srv):
            try:
                if obj is not None:
                    obj.close()
            except OSError:
                pass
        self.worker = self.worker_conn = self.worker_srv = None
        self.worker_link = None
        self.v_tel.set("viewer not running")
        self.v_mode.set("")
        self.b_launch.configure(state="normal")
        self.b_kill.configure(state="disabled")

    # ---- the one place widgets get updated ------------------------------
    def _pump(self):
        """Drain both queues on the MAIN thread and refresh the widgets."""
        # scenario.py stdout
        drained = 0
        while drained < 200:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            drained += 1
            if line is None:
                self._run_finished()
            else:
                self.log.insert("end", line)
                self.log.see("end")

        # viewer telemetry
        if self.worker_link is not None:
            last = None
            for msg in self.worker_link.poll():
                if "error" in msg:
                    self.live_log.insert("end", f"worker: {msg['error']}\n")
                    self.live_log.see("end")
                elif "info" in msg:
                    # What the worker ACTUALLY built, which can differ from
                    # the widgets if the tank overrode the scene's mass.
                    i = msg["info"]
                    mode = ("variable mass: "
                            f"{i['dry_mass_kg']:g} kg dry + "
                            f"{i['propellant_kg']:g} kg prop"
                            if i["variable_mass"] else "fixed mass")
                    self.v_mode.set(
                        f"{i['scene']}   {mode}   {i['mass_kg']:.3f} kg   "
                        f"T/W {i['twr']:.2f}"
                        + ("" if i["can_lift"] else "   CANNOT LIFT"))
                    self.live_log.insert(
                        "end", f"flying {i['scene']} / {i['controller']}: "
                               f"{mode}, mass {i['mass_kg']:.3f} kg "
                               f"(scene XML says {i['scene_xml_mass_kg']:g} kg), "
                               f"engine {i['thrust_max_n']:g} N, "
                               f"hover needs {i['weight_n']:.2f} N\n")
                    if not i["can_lift"]:
                        self.live_log.insert(
                            "end", "  ^ the engine is weaker than the craft's "
                                   "weight, so it cannot leave the ground and "
                                   "the target slider will look dead.\n")
                    self.live_log.see("end")
                else:
                    last = msg
            for err in self.worker_link.errors():
                self.live_log.insert("end", f"link: {err}\n")
                self.live_log.see("end")
            if last is not None:
                self.v_tel.set(
                    f"t {last['t']:7.2f} s   z {last['z']:7.3f} m   "
                    f"vz {last['vz']:+7.3f} m/s\n"
                    f"set {last['setpoint']:6.2f} m   thrust {last['thrust']:6.2f} N   "
                    f"mass {last['mass']:6.3f} kg   prop {last['prop']:5.3f} kg"
                    + ("" if last.get("running", True) else "   [PAUSED]"))
            if self.worker_link.closed or (self.worker and self.worker.poll() is not None):
                err = ""
                if self.worker and self.worker.stderr:
                    try:
                        err = self.worker.stderr.read() or ""
                    except (OSError, ValueError):
                        err = ""
                if err.strip():
                    self.live_log.insert("end", f"worker stderr: {err.strip()[:400]}\n")
                self._clear_worker()

        self.root.after(POLL_MS, self._pump)

    def on_close(self):
        """Closing the window must not orphan either child process."""
        if self.proc is not None:
            self.stop_run()
        self.kill_worker()
        self.root.destroy()


# =======================================================================
# SELF-TEST - no window, so this runs anywhere
# =======================================================================
def run_selftest() -> int:
    """Checks on everything that does not need a display.

        uv run python scripts/gui.py --selftest
    """
    bar = "=" * 74
    print(bar); print("GUI SELF-TEST"); print(bar)
    passed, failed = 0, []

    def truth(label, cond, detail=""):
        nonlocal passed
        print(f"  [{'ok  ' if cond else 'FAIL'}] {label:<46} {detail}")
        if cond:
            passed += 1
        else:
            failed.append(label)

    print("\n[1] discovery")
    scenes = find_scenes()
    truth("finds scenes", len(scenes) >= 7, f"{len(scenes)} scenes")
    truth("includes 08_thrust_1D_smol.xml",
          "08_thrust_1D_smol.xml" in scenes)
    ctrls = find_controllers()
    truth("finds the three control laws",
          {"hover", "hover_interactive", "vectored_translate"} <= set(ctrls),
          ", ".join(ctrls))
    truth("excludes set_target / world_to_body",
          "set_target" not in ctrls and "world_to_body" not in ctrls)

    # A scene dropped in the directory must appear with no code change.
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        (pathlib.Path(td) / "99_made_up.xml").write_text("<mujoco/>")
        truth("a new scene file is picked up automatically",
              find_scenes(td) == ["99_made_up.xml"])

    print("\n[2] command building")
    base = {"scene": "07_thrust_1D.xml", "controller": "hover",
            "target": "2.5", "descent_rate": "0.2", "control_hz": "",
            "propellant": "4", "dry_mass": "8", "isp": "57.8",
            "gravity": "", "label": "", "fixed_mass": False, "no_plot": True}
    cmd = build_scenario_cmd(base, python="PY", script="S")
    truth("controller is module-qualified",
          "example_hover:hover" in cmd, " ".join(cmd[2:8]))
    truth("scene is passed as a path", "scenes/07_thrust_1D.xml" in cmd)
    truth("set values become flags",
          "--target" in cmd and "2.5" in cmd and "--isp" in cmd)
    truth("blank fields are omitted entirely",
          "--control-hz" not in cmd and "--gravity" not in cmd)
    truth("--no-plot honoured", "--no-plot" in cmd)
    truth("--fixed-mass absent when unchecked", "--fixed-mass" not in cmd)
    cmd2 = build_scenario_cmd({**base, "fixed_mass": True}, python="PY", script="S")
    truth("--fixed-mass present when checked", "--fixed-mass" in cmd2)
    wcmd = build_worker_cmd(base, 4321, python="PY", script="W")
    truth("worker gets the port", "--port" in wcmd and "4321" in wcmd)
    truth("worker takes no scenario-only flags",
          "--descent-rate" not in wcmd and "--no-plot" not in wcmd)

    print("\n[2b] gains are attributed to the right law")
    import example_hover
    # Which gain boxes stay live is DERIVED from each law's code, not from
    # GAIN_GROUPS' labels, so a law added later is handled with no edit here.
    used_pd = gains_used_by("hover")
    used_pid = gains_used_by("hover_pid")
    used_cas = gains_used_by("hover_cascade")
    truth("hover reads the PD gains only",
          {"KP_ALT", "KD_ALT"} <= used_pd
          and not ({"KI_ALT", "KP_V"} & used_pd),
          ", ".join(sorted(n for n in GAIN_FIELDS if n in used_pd)))
    truth("hover_pid adds the integral gains",
          {"KI_ALT", "I_LIMIT_N"} <= used_pid and "KP_V" not in used_pid,
          ", ".join(sorted(n for n in GAIN_FIELDS if n in used_pid)))
    truth("hover_cascade reads the cascade gains, not KI_ALT",
          {"KP_POS_V", "KP_V", "V_MAX_MPS"} <= used_cas
          and "KI_ALT" not in used_cas,
          ", ".join(sorted(n for n in GAIN_FIELDS if n in used_cas)))
    truth("every GAIN_GROUPS name is a real settable gain",
          set(GAIN_FIELDS) <= set(example_hover._GAIN_NAMES))
    # The grouping is only a label; the greying must come from the code. A
    # law this table has never heard of still gets attributed correctly.
    example_hover._SELFTEST_GAIN = 0.0
    try:
        example_hover._GAIN_NAMES += ("_SELFTEST_GAIN",)
        src = ("def _selftest_law(state):\n"
               "    return KP_V * _SELFTEST_GAIN\n")
        exec(compile(src, "<selftest>", "exec"), vars(example_hover))
        found = gains_used_by("_selftest_law")
        truth("a law absent from GAIN_GROUPS is still attributed",
              found == {"KP_V", "_SELFTEST_GAIN"}, ", ".join(sorted(found)))
    finally:
        example_hover._GAIN_NAMES = tuple(
            n for n in example_hover._GAIN_NAMES if n != "_SELFTEST_GAIN")
        del example_hover._SELFTEST_GAIN
        del vars(example_hover)["_selftest_law"]
    truth("an unknown law greys nothing (fails open)",
          gains_used_by("no_such_law") == set(example_hover._GAIN_NAMES))
    # A gain the law cannot read must not be recorded as if it mattered.
    class _V:
        def __init__(self, v): self.v = v
        def get(self): return self.v
    allg = {n: _V("1.5") for n in GAIN_FIELDS}
    spec_cas = App.gain_spec(allg, used_cas)
    truth("gain_spec drops gains the chosen law ignores",
          "KI_ALT" not in spec_cas and "KP_V=1.5" in spec_cas, spec_cas)
    truth("gain_spec unfiltered still passes everything",
          App.gain_spec(allg).count("=") == len(GAIN_FIELDS))

    print("\n[2c] fixed mass drops the tank-only flags")
    # scenario.py / sim_worker.py build no PropellantTank with --fixed-mass,
    # so --propellant/--dry-mass/--isp were accepted and silently ignored -
    # and landed in run.json as if they had applied.
    fixed = build_scenario_cmd({**base, "fixed_mass": True},
                               python="PY", script="S")
    truth("variable mass passes all three tank flags",
          all(f in cmd for f in ("--propellant", "--dry-mass", "--isp")))
    truth("fixed mass passes none of them",
          not any(f in fixed for f in ("--propellant", "--dry-mass", "--isp")),
          " ".join(fixed[4:]))
    truth("fixed mass keeps the non-tank flags",
          "--target" in fixed and "--descent-rate" in fixed)
    wfixed = build_worker_cmd({**base, "fixed_mass": True}, 1,
                              python="PY", script="W")
    truth("worker cmd drops them too",
          not any(f in wfixed for f in ("--propellant", "--dry-mass", "--isp")))
    truth("MASS_FIELDS names match the params() keys",
          set(MASS_FIELDS) <= set(base))
    truth("the greying helpers exist and are wired to both tabs",
          hasattr(App, "_update_gain_fields")
          and hasattr(App, "_update_mass_fields")
          and hasattr(App, "_on_mass_mode"))

    print("\n[2d] the record-mp4 checkbox")
    truth("--video absent when unchecked", "--video" not in cmd)
    vid = build_scenario_cmd({**base, "video": True}, python="PY", script="S")
    truth("--video present when checked", "--video" in vid)
    truth("--video is scenario-only, never sent to the worker",
          "--video" not in build_worker_cmd({**base, "video": True}, 1,
                                            python="PY", script="W"))
    cam = build_scenario_cmd({**base, "video": True, "camera": "chase"},
                             python="PY", script="S")
    truth("a camera choice is passed through",
          "--camera" in cam and "chase" in cam)
    # Idle thrusters are hidden by DEFAULT, so the flag must be absent
    # unless the box is ticked.
    truth("--all-thrusters absent by default", "--all-thrusters" not in cmd)
    truth("--all-thrusters present when ticked",
          "--all-thrusters" in build_scenario_cmd(
              {**base, "all_thrusters": True}, python="PY", script="S"))

    print("\n[3] metric diffing")
    a = {"peak_altitude_m": 2.0, "overshoot_m": 0.05, "timeouts": []}
    b = {"peak_altitude_m": 3.0, "overshoot_m": 0.05, "timeouts": ["ASCEND"]}
    rows = diff_metrics(a, b)
    d = {k: differs for k, _, _, differs in rows}
    truth("spots a changed number", d["peak_altitude_m"])
    truth("leaves an equal number alone", not d["overshoot_m"])
    truth("spots a changed list", d["timeouts"])
    truth("keeps METRIC_ORDER ordering",
          [k for k, *_ in rows][:2] == ["peak_altitude_m", "overshoot_m"])
    rows2 = diff_metrics({"only_in_a": 1.0}, {"only_in_b": 2.0})
    truth("unions keys from both runs",
          {k for k, *_ in rows2} == {"only_in_a", "only_in_b"})
    truth("missing values render as '-'", fmt_metric(None) == "-")

    print("\n[4] run discovery and labelling")
    runs = find_runs()
    truth("find_runs survives an empty/missing outputs/", isinstance(runs, list),
          f"{len(runs)} runs")
    if runs:
        truth("labels a run readably", len(run_label(runs[0])) > 10,
              run_label(runs[0])[:58])
    with tempfile.TemporaryDirectory() as td:
        bad = pathlib.Path(td) / "corrupt_run"
        bad.mkdir()
        (bad / "run.json").write_text("{not json")
        got = find_runs(td)
        truth("a corrupt run.json does not crash discovery",
              len(got) == 1 and got[0]["meta"] == {})

    print("\n[4b] lift capability is computed, not assumed")
    # The tank's dry+propellant REPLACES the scene's mass, so a light scene
    # with heavy tank defaults is pinned to the ground: the engine saturates,
    # nothing moves, and the target slider looks broken. Reported before
    # launch now instead of being discovered the hard way.
    smol = scene_capability("08_thrust_1D_smol.xml", None)
    truth("smol scene lifts on its own mass",
          smol["can_lift"] and smol["mass"] == 1.0,
          f"{smol['mass']:g} kg, T/W {smol['twr']:.2f}")
    bad = scene_capability("08_thrust_1D_smol.xml", 12.0)
    truth("smol + 12 kg tank defaults CANNOT lift",
          not bad["can_lift"], f"T/W {bad['twr']:.2f} (needs > 1)")
    truth("override does not change the scene's own XML mass",
          bad["xml_mass"] == 1.0, f"xml {bad['xml_mass']:g} kg")
    big = scene_capability("07_thrust_1D.xml", 12.0)
    truth("07 + 12 kg tank defaults lifts fine",
          big["can_lift"], f"T/W {big['twr']:.2f}")
    gsrc_live = (HERE / "gui.py").read_text()
    truth("Live tab has its own scene selector",
          "self.lv_scene" in gsrc_live and "def live_params" in gsrc_live)
    truth("Live tab exposes the mass mode",
          "self.lv_massmode" in gsrc_live)
    truth("worker reports can_lift so the GUI can show it",
          '"can_lift"' in (HERE / "sim_worker.py").read_text())

    print("\n[5] run directories never collide")
    # scenario.py's stamp resolves to one second, and it used to
    # mkdir(exist_ok=True) - so two runs started in the same second
    # overwrote each other's files. Found by clicking Run twice from this
    # GUI. Assert the suffixing that replaced it.
    import re
    src = (HERE / "scenario.py").read_text()
    truth("scenario.py does not mkdir(exist_ok=True) for a run dir",
          "rundir.mkdir(parents=True, exist_ok=False)" in src)
    truth("scenario.py suffixes a colliding run dir",
          re.search(r'alt = parent / f"\{name\}_\{n\}"', src) is not None)

    print("\n[6] slider binding")
    # tk.Scale.set() does NOT fire the widget's `command` - only a user drag
    # does - so binding commands to `command` meant programmatic changes
    # silently never reached the worker. Must go through a variable trace.
    gsrc = (HERE / "gui.py").read_text()
    # Check the live widget, not the source text - grepping gui.py for the
    # old binding matches this very assertion.
    truth("sliders use a variable trace, not Scale(command=...)",
          "self.var_target.trace_add" in gsrc
          and not hasattr(App, "_on_target_slider"))
    # live_params must read the slider's var, so the launched worker starts
    # at whatever the slider shows rather than a Run-tab entry box.
    truth("live_params takes its target from the slider var",
          "self.var_target.get()" in gsrc)

    print("\n[7] worker lifecycle contract")
    wsrc = (HERE / "sim_worker.py").read_text()
    truth("worker exits via os._exit (viewer teardown segfaults)",
          "os._exit(0)" in wsrc)
    truth("worker re-baselines next_sync on resume/reset",
          wsrc.count("next_sync = 0.0") >= 2)
    truth("kill_worker escalates quit -> terminate -> kill",
          all(k in gsrc for k in ('{"cmd": "quit"}', ".terminate()", ".kill()")))
    lsrc = (HERE / "simlink.py").read_text()
    truth("LineReader catches BlockingIOError only",
          "except BlockingIOError:" in lsrc
          and "except Exception:\n            pass" not in lsrc)

    print("\n[2e] the valve layer controls")
    vbase = {**base, "valve": True, "valve_curve": "sqrt",
             "valve_feed": "regulated", "valve_tank_l": "12",
             "valve_tau": "0.01", "controller": "hover_valve"}
    cv = build_scenario_cmd(vbase, python="PY", script="S")
    truth("--valve is passed when the box is checked", "--valve" in cv)
    for flag, val in (("--valve-curve", "sqrt"), ("--valve-feed", "regulated"),
                      ("--valve-tank-l", "12"), ("--valve-tau", "0.01")):
        truth(f"  {flag} is passed",
              flag in cv and cv[cv.index(flag) + 1] == val)
    # OFF, the four settings must be DROPPED, not merely ignored. Passed and
    # ignored is how a fixed-mass run came to record a 4 kg tank in
    # run.json, and the same trap applies to a curve name.
    coff = build_scenario_cmd({**vbase, "valve": False},
                              python="PY", script="S")
    truth("  with the valve off, --valve is absent", "--valve" not in coff)
    truth("    and so is every valve setting, not just ignored",
          not any(a.startswith("--valve") for a in coff),
          " ".join(a for a in coff if a.startswith("--valve")) or "none")
    truth("  VALVE_FIELDS names match the params() keys",
          set(VALVE_FIELDS) <= set(vbase))
    # The Live tab has no valve: sim_worker.py does not build one, so
    # offering it there would be a flag that goes nowhere.
    wv = build_worker_cmd(vbase, 1, python="PY", script="S")
    truth("  the live worker is never given valve flags",
          not any(a.startswith("--valve") for a in wv))
    # Discovery, not a hand-kept list - the same rule as find_scenes.
    truth("the curve family is read from valve.py",
          set(valve_curves()) >= {"ball", "linear", "reversed"},
          ", ".join(valve_curves()))
    truth("the valve laws are read from the controller module",
          "hover_valve" in valve_laws() and "hover" not in valve_laws(),
          ", ".join(valve_laws()))

    print("\n[8] the interactive figure view")
    # The toolbar is the whole point: pan, box-zoom and Home are what make a
    # plot interrogable. Asserted so a matplotlib release that renames or
    # drops one is caught here rather than by a user with a dead button.
    try:
        from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
        tools = [t[0] for t in NavigationToolbar2Tk.toolitems if t[0]]
        truth("the toolbar offers Home, Pan, Zoom and Save",
              {"Home", "Pan", "Zoom", "Save"} <= set(tools),
              ", ".join(tools))
    except Exception as e:
        truth("the toolbar offers Home, Pan, Zoom and Save", False,
              f"{type(e).__name__}: {e}")

    # Scroll-to-zoom, exercised with no window: the handler is pure axis
    # arithmetic, so a stub canvas and a stub event reach all of it.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as _plt

    class _StubCanvas:
        def __init__(self):
            self.handler = None
            self.draws = 0
        def mpl_connect(self, name, fn):
            assert name == "scroll_event", name
            self.handler = fn
        def draw_idle(self):
            self.draws += 1

    class _StubEvent:
        def __init__(self, ax, step, xdata):
            self.inaxes, self.step, self.xdata = ax, step, xdata
            self.button = "up" if step > 0 else "down"

    zfig, zax = _plt.subplots()
    zax.set_xlim(0.0, 10.0)
    cv = _StubCanvas()
    App._bind_scroll_zoom(cv, zfig, factor=2.0)
    truth("scroll-to-zoom binds a scroll_event handler", cv.handler is not None)
    cv.handler(_StubEvent(zax, 1.0, 2.0))
    lo, hi = zax.get_xlim()
    # factor 2.0, one notch up -> the window halves, and the point under the
    # cursor must not move: that is what makes scrolling feel like zooming
    # rather than scrolling.
    truth("  scrolling up narrows the time window",
          abs((hi - lo) - 5.0) < 1e-9, f"{lo:.3f}..{hi:.3f}")
    truth("  the point under the cursor stays put",
          abs(lo - 1.0) < 1e-9 and abs(hi - 6.0) < 1e-9,
          f"{lo:.3f}..{hi:.3f}")
    cv.handler(_StubEvent(zax, -1.0, 2.0))
    lo2, hi2 = zax.get_xlim()
    truth("  scrolling down is its exact inverse",
          abs(lo2 - 0.0) < 1e-9 and abs(hi2 - 10.0) < 1e-9,
          f"{lo2:.3f}..{hi2:.3f}")
    truth("  each notch asks for a redraw", cv.draws == 2, f"{cv.draws} draws")
    # Outside the axes, and on a NaN cursor position, it must do nothing
    # rather than raise - scrolling over the figure margin is normal.
    before = zax.get_xlim()
    cv.handler(_StubEvent(None, 1.0, 2.0))
    cv.handler(_StubEvent(zax, 1.0, None))
    truth("  a scroll off the axes changes nothing",
          zax.get_xlim() == before and cv.draws == 2)
    _plt.close(zfig)

    # Panels within one figure already share x, so one scroll moves them all.
    # This is what makes a single handler enough.
    import scenario as _scn
    runs_av = find_runs()
    plotted = [r for r in runs_av if (r["path"] / "flight.csv").is_file()]
    if plotted:
        f1, _ = _scn.figure_for_run(plotted[0]["path"], config=False)
        truth("a rebuilt figure has the full panel stack",
              len(f1.axes) >= 5, f"{len(f1.axes)} axes")
        f1.axes[0].set_xlim(3.0, 7.0)
        truth("  every panel shares the time axis",
              all(tuple(a.get_xlim()) == (3.0, 7.0) for a in f1.axes[:5]),
              ", ".join(f"{a.get_xlim()[0]:.0f}-{a.get_xlim()[1]:.0f}"
                        for a in f1.axes[:5]))
        if len(plotted) >= 2:
            f2, _ = _scn.figure_for_run(plotted[1]["path"], config=False)
            # sharex across SEPARATE figures is what links the Compare pair.
            f1.axes[0].sharex(f2.axes[0])
            f1.axes[0].set_xlim(2.0, 4.0)
            truth("  two compared figures pan as one",
                  tuple(f2.axes[0].get_xlim()) == (2.0, 4.0),
                  f"{f2.axes[0].get_xlim()}")
            _plt.close(f2)
        _plt.close(f1)

    # Backend ordering. _select_tk_backend has to run BEFORE scenario is
    # imported and a figure built, because plot_flight only leaves the
    # backend alone if something interactive is already selected, and
    # matplotlib.use after pyplot has drawn is a no-op.
    import inspect as _insp
    for meth in ("_show_figure_window", "_show_both_interactive"):
        body = _insp.getsource(getattr(App, meth))
        i_sel = body.find("_select_tk_backend")
        i_imp = body.find("import scenario")
        truth(f"{meth} selects TkAgg before importing scenario",
              0 <= i_sel < i_imp, f"{i_sel} < {i_imp}")
        truth(f"  {meth} closes its figure(s) on window close",
              "WM_DELETE_WINDOW" in body and "plt.close" in body)

    # The static path is kept precisely so a matplotlib failure degrades to
    # a picture instead of to nothing.
    for meth, fb in (("open_plot", "_show_png_window"),
                     ("show_both", "_show_both_png")):
        body = _insp.getsource(getattr(App, meth))
        truth(f"{meth} falls back to the PNG if the figure fails",
              "except Exception" in body and fb in body)
    truth("flight.png is still written on every run",
          "fig.savefig(path" in (HERE / "scenario.py").read_text())
    # The teardown warning is filtered in _select_tk_backend, not around a
    # single draw(): the draws that trip it are deferred into tkinter's own
    # dispatcher, where no local catch_warnings can reach them.
    sel_src = _insp.getsource(App._select_tk_backend)
    truth("the transient constrained_layout warning is filtered",
          "constrained_layout not applied" in sel_src
          and "filterwarnings" in sel_src)
    # Panels at a realistic window size are nowhere near collapsed, which
    # is why the warning is noise and not a sizing bug worth chasing.
    if plotted:
        fchk, _ = _scn.figure_for_run(plotted[0]["path"], config=False)
        fchk.set_size_inches(12 / 1.1, 7 / 1.1)
        fchk.canvas.draw()
        hs = [a.get_position().height for a in fchk.axes[:5]]
        truth("  panels stay well clear of zero height at 1200x700",
              min(hs) > 0.05, f"min {min(hs):.3f} of figure height")
        _plt.close(fchk)
    # One reader, in scenario.py next to the code that wrote the files, so
    # the writer and the reader of a run cannot drift apart. The GUI only
    # ever asks whether the CSV exists; parsing it is scenario.load_run's
    # job, and a second parser here would be the drift.
    app_src = gsrc.split("def run_selftest")[0]
    csv_uses = [ln.strip() for ln in app_src.splitlines()
                if "flight.csv" in ln and not ln.lstrip().startswith("#")]
    truth("the GUI does not parse flight.csv itself",
          "import csv" not in app_src
          and all(".is_file()" in ln for ln in csv_uses),
          "; ".join(csv_uses) or "never named outside comments")
    truth("  parsing lives in scenario.load_run",
          "import csv as _csv" in (HERE / "scenario.py").read_text())

    print()
    print(bar)
    total = passed + len(failed)
    if failed:
        print(f"FAILED  {len(failed)}/{total}")
        for f in failed:
            print(f"  - {f}")
        print(bar)
        return 1
    print(f"ALL {total} CHECKS PASSED")
    print(bar)
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--demo", action="store_true",
                   help="bigger fonts, less clutter, for projecting")
    p.add_argument("--selftest", action="store_true",
                   help="headless checks, no window")
    args = p.parse_args()

    if args.selftest:
        return run_selftest()

    import tkinter as tk
    try:
        root = tk.Tk()
    except tk.TclError as e:
        print(f"error: no display available ({e})\n"
              f"  This needs a GUI session. On WSL that means WSLg; try\n"
              f"  `echo $DISPLAY` and `ls /mnt/wslg`. For headless checks:\n"
              f"      uv run python scripts/gui.py --selftest", file=sys.stderr)
        return 1
    App(root, demo=args.demo)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
