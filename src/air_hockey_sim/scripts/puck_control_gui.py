#!/usr/bin/env python3
import math
import os
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH_SCRIPT = os.path.join(SCRIPT_DIR, "launch_puck.sh")
RESET_SCRIPT = os.path.join(SCRIPT_DIR, "reset_puck.sh")

# CAD-frame center of the table (see air_hockey_table.sdf) - matches the
# puck's spawn pose.
CENTER_X, CENTER_Y = -0.99, -0.5325

# Screen "up" = +Y (CAD), screen "right" = +X (CAD). Arbitrary but fixed
# and labeled, since the table has no inherent on-screen orientation.
DIRECTIONS = {
    "nw": (-1, 1), "n": (0, 1), "ne": (1, 1),
    "w": (-1, 0),               "e": (1, 0),
    "sw": (-1, -1), "s": (0, -1), "se": (1, -1),
}
ARROWS = {
    "nw": "↖", "n": "↑ +Y", "ne": "↗",
    "w": "← −X", "e": "+X →",
    "sw": "↙", "s": "↓ −Y", "se": "↘",
}


class PuckControlGUI:
    def __init__(self, root):
        self.root = root
        root.title("Puck Control")
        root.resizable(False, False)

        main = ttk.Frame(root, padding=12)
        main.grid()

        controls = ttk.LabelFrame(main, text="Shove strength", padding=8)
        controls.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))

        ttk.Label(controls, text="Speed (m/s)").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.DoubleVar(value=1.5)
        self.speed_label = ttk.Label(controls, text="1.5")
        ttk.Scale(
            controls, from_=0.2, to=4.0, orient="horizontal", variable=self.speed_var,
            command=lambda v: self.speed_label.config(text=f"{float(v):.1f}"),
            length=220,
        ).grid(row=0, column=1, padx=6)
        self.speed_label.grid(row=0, column=2)

        # --- 3x3 compass of direction buttons, reset in the middle ---
        compass = ttk.LabelFrame(main, text="Hit direction  /  Reset", padding=8)
        compass.grid(row=1, column=0, columnspan=3)

        positions = {
            "nw": (0, 0), "n": (0, 1), "ne": (0, 2),
            "w": (1, 0), "e": (1, 2),
            "sw": (2, 0), "s": (2, 1), "se": (2, 2),
        }
        self.direction_buttons = []
        for key, (r, c) in positions.items():
            btn = ttk.Button(
                compass, text=ARROWS[key], width=8,
                command=lambda k=key: self.on_launch(k),
            )
            btn.grid(row=r, column=c, padx=3, pady=3)
            self.direction_buttons.append(btn)

        # Reset is never disabled - it should always be able to interrupt
        # an in-flight launch, not queue up behind it.
        reset_btn = tk.Button(
            compass, text="⟳\nRESET", width=8, height=3,
            bg="#c0392b", fg="white", activebackground="#e74c3c",
            command=self.on_reset,
        )
        reset_btn.grid(row=1, column=1, padx=3, pady=3)

        # --- status log ---
        log_frame = ttk.LabelFrame(main, text="Status", padding=6)
        log_frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        self.log = tk.Text(log_frame, width=48, height=7, state="disabled", wrap="word")
        self.log.grid()

        self._log("Ready. Sim must already be running "
                   "(ros2 launch air_hockey_sim air_hockey_table.launch.py).")

        # Guards against overlapping launch_puck.sh calls - each is now a
        # single instantaneous-impulse call (under ~1s typically), but
        # direction buttons still get disabled while one is in flight so
        # rapid clicks can't stack multiple impulses in confusing ways.
        # Reset stays enabled so it can always interrupt one instead of
        # queuing behind it.
        self._busy_count = 0

    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _set_direction_buttons_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for btn in self.direction_buttons:
            btn.config(state=state)

    def _run_async(self, cmd, description, disable_directions=False):
        self._log("> " + description)
        if disable_directions:
            self._busy_count += 1
            self._set_direction_buttons_enabled(False)

        def done():
            if disable_directions:
                self._busy_count -= 1
                if self._busy_count <= 0:
                    self._busy_count = 0
                    self._set_direction_buttons_enabled(True)

        def worker():
            try:
                # launch_puck.sh makes one `ign topic` call (2s timeout);
                # reset_puck.sh makes up to three (1s timeout each). This
                # is a generous ceiling above either's worst case, not the
                # expected duration - a normal call finishes in ~1s.
                result = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=8)
                out = (result.stdout or "").strip()
                err = (result.stderr or "").strip()
                if out:
                    self.root.after(0, self._log, out)
                if result.returncode != 0:
                    self.root.after(0, self._log, f"[error] {err or 'command failed'}")
            except Exception as exc:  # noqa: BLE001 - surface any failure in the GUI
                self.root.after(0, self._log, f"[error] {exc}")
            finally:
                self.root.after(0, done)

        threading.Thread(target=worker, daemon=True).start()

    def on_launch(self, direction_key):
        dx, dy = DIRECTIONS[direction_key]
        norm = math.hypot(dx, dy)
        speed = self.speed_var.get()
        vx = dx / norm * speed
        vy = dy / norm * speed
        self._run_async(
            [LAUNCH_SCRIPT, f"{vx:.3f}", f"{vy:.3f}"],
            f"launch_puck.sh {vx:.2f} {vy:.2f}",
            disable_directions=True,
        )

    def on_reset(self):
        self._run_async(
            [RESET_SCRIPT, f"{CENTER_X}", f"{CENTER_Y}"],
            f"reset_puck.sh {CENTER_X} {CENTER_Y}",
        )


def main():
    root = tk.Tk()
    PuckControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
