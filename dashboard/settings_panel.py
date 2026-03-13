"""
settings_panel.py — Scrollable settings panel with grouped parameter widgets.

Buttons: Read from Car, Write to Car, Save to EEPROM, Load File, Save File.
Grouped Spinbox/Entry widgets for all 25 parameters.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from car_config import (
    DEFAULTS, KEY_NAMES, READONLY_KEYS, PARAM_GROUPS, FLOAT_KEYS,
)
from protocol import encode_get, encode_set, encode_save, encode_load, encode_reset
from settings_file import save_settings, load_settings


class SettingsPanel:
    """Scrollable settings panel with grouped parameter widgets."""

    def __init__(self, parent, connection):
        self.conn = connection
        self._vars = {}      # key → tk.StringVar
        self._widgets = {}   # key → widget

        # ── Outer frame with scrollbar ──────────────────────────────────
        self.frame = ttk.Frame(parent)
        self._canvas = tk.Canvas(self.frame, width=280, highlightthickness=0)
        self._scrollbar = ttk.Scrollbar(self.frame, orient="vertical",
                                         command=self._canvas.yview)
        self._inner = ttk.Frame(self._canvas)

        self._inner.bind("<Configure>",
                          lambda e: self._canvas.configure(
                              scrollregion=self._canvas.bbox("all")))
        self._canvas.create_window((0, 0), window=self._inner, anchor="nw")
        self._canvas.configure(yscrollcommand=self._scrollbar.set)

        self._scrollbar.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

        # Mousewheel scrolling
        self._canvas.bind_all("<MouseWheel>", self._on_mousewheel, add="+")

        # ── Buttons row ─────────────────────────────────────────────────
        btn_frame = ttk.Frame(self._inner)
        btn_frame.pack(fill="x", padx=4, pady=(4, 2))

        ttk.Button(btn_frame, text="Read", width=6,
                    command=self.read_from_car).pack(side="left", padx=1)
        ttk.Button(btn_frame, text="Write", width=6,
                    command=self.write_to_car).pack(side="left", padx=1)
        ttk.Button(btn_frame, text="Save EE", width=7,
                    command=self.save_eeprom).pack(side="left", padx=1)
        ttk.Button(btn_frame, text="Reset", width=6,
                    command=self.reset_defaults).pack(side="left", padx=1)

        btn_frame2 = ttk.Frame(self._inner)
        btn_frame2.pack(fill="x", padx=4, pady=(0, 4))

        ttk.Button(btn_frame2, text="Load File", width=9,
                    command=self.load_file).pack(side="left", padx=1)
        ttk.Button(btn_frame2, text="Save File", width=9,
                    command=self.save_file).pack(side="left", padx=1)
        ttk.Button(btn_frame2, text="Load EE", width=7,
                    command=self.load_eeprom).pack(side="left", padx=1)

        # ── Parameter groups ────────────────────────────────────────────
        for group_name, keys in PARAM_GROUPS.items():
            lf = ttk.LabelFrame(self._inner, text=group_name)
            lf.pack(fill="x", padx=4, pady=2)

            for key in keys:
                row = ttk.Frame(lf)
                row.pack(fill="x", padx=4, pady=1)

                label_text = KEY_NAMES.get(key, key)
                ttk.Label(row, text=label_text, width=20,
                           anchor="w").pack(side="left")

                var = tk.StringVar(value=str(DEFAULTS.get(key, "")))
                self._vars[key] = var

                readonly = key in READONLY_KEYS
                if key == "RCW":
                    # Boolean checkbox
                    bool_var = tk.IntVar(value=int(DEFAULTS.get(key, 1)))
                    cb = ttk.Checkbutton(row, variable=bool_var)
                    cb.pack(side="right")
                    # Link bool_var to string var
                    bool_var.trace_add("write",
                        lambda *a, bv=bool_var, sv=var: sv.set(str(bv.get())))
                    var.set(str(bool_var.get()))
                    self._widgets[key] = bool_var
                    if readonly:
                        cb.configure(state="disabled")
                else:
                    entry = ttk.Entry(row, textvariable=var, width=10)
                    entry.pack(side="right")
                    self._widgets[key] = entry
                    if readonly:
                        entry.configure(state="readonly")

    def _on_mousewheel(self, event):
        self._canvas.yview_scroll(-1 * (event.delta // 120), "units")

    def get_values(self) -> dict:
        """Read current widget values as a dict {key: value}."""
        result = {}
        for key, var in self._vars.items():
            raw = var.get().strip()
            try:
                if key in FLOAT_KEYS:
                    result[key] = float(raw)
                else:
                    result[key] = int(float(raw))
            except ValueError:
                result[key] = raw
        return result

    def set_values(self, params: dict):
        """Update widget values from a dict {key: value}."""
        for key, value in params.items():
            if key in self._vars:
                self._vars[key].set(str(value))
                # Update checkbox if RCW
                if key == "RCW" and key in self._widgets:
                    self._widgets[key].set(int(value))

    # ── Car communication ────────────────────────────────────────────────

    def read_from_car(self):
        """Send $GET to read all parameters from car."""
        if not self.conn.connected:
            messagebox.showwarning("Not Connected", "Connect to car first.")
            return
        self.conn.send_command(encode_get())

    def write_to_car(self):
        """Send $SET with all writable parameters."""
        if not self.conn.connected:
            messagebox.showwarning("Not Connected", "Connect to car first.")
            return
        params = {k: v for k, v in self.get_values().items()
                  if k not in READONLY_KEYS}
        self.conn.send_command(encode_set(params))

    def save_eeprom(self):
        """Send $SAVE to persist settings to EEPROM."""
        if not self.conn.connected:
            messagebox.showwarning("Not Connected", "Connect to car first.")
            return
        self.conn.send_command(encode_save())

    def load_eeprom(self):
        """Send $LOAD to load settings from EEPROM."""
        if not self.conn.connected:
            messagebox.showwarning("Not Connected", "Connect to car first.")
            return
        self.conn.send_command(encode_load())

    def reset_defaults(self):
        """Send $RST to reset to compile-time defaults."""
        if not self.conn.connected:
            messagebox.showwarning("Not Connected", "Connect to car first.")
            return
        self.conn.send_command(encode_reset())

    # ── File I/O ─────────────────────────────────────────────────────────

    def load_file(self):
        """Load settings from a JSON file."""
        path = filedialog.askopenfilename(
            title="Load Settings",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            params = load_settings(path)
            self.set_values(params)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))

    def save_file(self):
        """Save current settings to a JSON file."""
        path = filedialog.asksaveasfilename(
            title="Save Settings",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            save_settings(self.get_values(), path)
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def handle_cfg_response(self, params: dict):
        """Handle a $CFG response — update all widgets."""
        self.set_values(params)
