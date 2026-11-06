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
                      ("--label", "label")):
        v = params.get(key)
        if v not in (None, ""):
            cmd += [flag, str(v)]
    if params.get("fixed_mass"):
        cmd.append("--fixed-mass")
    if params.get("no_plot"):
        cmd.append("--no-plot")
    return cmd


def build_worker_cmd(params, port, python=None, script=None):
    """argv for sim_worker.py, the live-viewer process."""
    py = python or sys.executable
    sc = script or str(HERE / "sim_worker.py")
    cmd = [py, sc, "--port", str(port),
           "--controller", f"{CONTROLLER_MODULE}:{params['controller']}",
           "--scene", f"scenes/{params['scene']}"]
    for flag, key in (("--target", "target"),
                      ("--propellant", "propellant"),
                      ("--dry-mass", "dry_mass"),
                      ("--isp", "isp"),
                      ("--gravity", "gravity")):
        v = params.get(key)
        if v not in (None, ""):
            cmd += [flag, str(v)]
    if params.get("fixed_mass"):
        cmd.append("--fixed-mass")
    return cmd


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
                "fixed_mass": bool(self.v_fixed.get()),
                "no_plot": False}

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
        def field(col, label, var, width=9):
            f = ttk.Frame(self.grid_params)
            f.grid(row=0, column=col, sticky="w", padx=(0, 10))
            ttk.Label(f, text=label).pack(anchor="w")
            ttk.Entry(f, textvariable=var, width=width).pack()
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
        field(3, "propellant [kg]", self.v_prop)
        field(4, "dry mass [kg]", self.v_dry)
        field(5, "Isp [s]", self.v_isp)
        field(6, "gravity", self.v_grav)
        field(7, "label", self.v_label, 12)
        ttk.Checkbutton(self.grid_params, text="fixed mass",
                        variable=self.v_fixed).grid(row=0, column=8, sticky="w")

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
        png = run["path"] / "flight.png"
        if not png.is_file():
            self.v_status.set("that run has no flight.png")
            return
        self._show_png_window(png, run["name"])

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
        self.cmp_out.insert("end", f"A  {a['name']}\nB  {b['name']}\n\n", "hdr")
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

        ttk.Label(sl, text="propellant [kg]").grid(row=1, column=0, sticky="w")
        self.var_prop = tk.DoubleVar(value=4.0)
        self.s_prop = tk.Scale(sl, from_=0.0, to=8.0, resolution=0.05,
                               orient="horizontal", length=380,
                               variable=self.var_prop)
        self.s_prop.grid(row=1, column=1, sticky="w")
        self.var_prop.trace_add("write", self._on_prop_var)

        self.v_tel = tk.StringVar(value="viewer not running")
        ttk.Label(self.tab_live, textvariable=self.v_tel,
                  font=("TkFixedFont", self.base + (6 if self.demo else 2)),
                  padding=10).pack(anchor="w")
        self.live_log = tk.Text(self.tab_live, height=6, font=self.mono)
        self.live_log.pack(fill="both", expand=True, padx=8, pady=8)
        if self.demo:
            self.live_log.pack_forget()

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
        cmd = build_worker_cmd(self.params(), port)
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
        self.var_target.set(float(self.params()["target"] or 2.0))
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
    truth("launch_worker seeds the target through the var",
          "self.var_target.set(" in gsrc)

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
