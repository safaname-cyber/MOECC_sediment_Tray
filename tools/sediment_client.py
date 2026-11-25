#!/usr/bin/env python3
"""
sediment_client.py — terminal client for the EyeROV Sediment Tray
(FW 1.7.0)
over the UART-to-Ethernet converter (ESP32 UART1, GPIO17/18).

Comms safety (FW 2.0.0 [DEV-PING]): the tray sends {"cmd":"ping","seq":N}
  every 15 s; this client answers {"cmd":"pong","seq":N} automatically.
  3 missed pongs → COMMS_LOST. (Older FW: see below.)

Comms safety (firmware [COMMS-RX-WDT]):
  The firmware latches COMMS_LOST if it sees no valid command for
  3 consecutive 10 s windows. This client sends {"cmd":"ping"} every
  --ping seconds (default 2 s) from a background thread for as long as it
  runs, so the link stays healthy while you type. Pongs are hidden unless
  verbose mode is on; a warning is printed if pongs stop arriving.
  After a COMMS_LOST fault, the client keeps pinging; send `rf`
  (reset_fault) to clear it — the firmware never clears it by itself.

Usage:
  python3 sediment_client.py                                (TCP 192.168.2.153:2000)
  python3 sediment_client.py --host <IP> --port <PORT>      (other converter)
  python3 sediment_client.py --host <IP> --port <PORT> --udp

Standard library only. Type `help` once connected.
"""

import argparse
import json
import socket
import sys
import threading
import time

try:
    import readline  # noqa: F401  (line editing + history)
except ImportError:  # Windows without pyreadline
    readline = None

PROMPT = "tray> "

# ----------------------------------------------------------------------------
# Short keys → firmware commands.
#   alias: (cmd, [(json_field, type, required), ...], description)
# ----------------------------------------------------------------------------
SHORTCUTS = {
    # Motion
    "h":      ("home",         [], "home (drive to CLOSED limit, lock)"),
    "o":      ("open",         [], "open tray (needs homed + closed_locked)"),
    "c":      ("close",        [], "close tray and lock"),
    "hl":     ("halt",         [], "halt: torque off, mode idle"),
    # Bench / assembly (firmware built with MOTION_BENCH_JOG 1, else unknown_cmd)
    "g":      ("goto",         [("pos", int, True)], "g <0..4095>  run until encoder count reached (or switch)"),
    "j":      ("jog",          [("steps", int, True)], "j <±counts>  + toward OPEN, - toward CLOSED (stops at switch)"),
    # Safety
    "x":      ("estop",        [], "E-STOP (latched fault)"),
    "rf":     ("reset_fault",  [], "reset latched fault"),
    "sr":     ("servo_reboot", [], "reboot servo"),
    "lk":     ("lock",         [], "solenoid lock ON (manual)"),
    "ul":     ("unlock",       [], "solenoid unlock (manual)"),
    # Query
    "s":      ("status",       [], "status"),
    "d":      ("diag",         [], "full diagnostics"),
    "v":      ("version",      [], "firmware version"),
    "w":      ("weigh",        [], "current weight"),
    "p":      ("pos_query",    [], "servo position"),
    "md":     ("motor_diag",   [], "motor V/I/T"),
    "ss":     ("servo_status", [], "servo status flags"),
    "st":     ("selftest",     [], "self test"),
    "n":      ("nop",          [], "nop (ack)"),
    "ping":   ("ping",         [], "manual ping"),
    # Load cell — legacy
    "rc":     ("raw_counts",   [], "last raw HX711 counts"),
    "gs":     ("get_scale",    [], "scale (integer, legacy)"),
    "sc":     ("set_scale",    [("scale_cpg", int, True)], "sc <N>  legacy integer scale, saved at once"),
    # Load cell — calibration (grabber-style)
    "t":      ("tare",         [("samples", int, False)], "t [samples]  tare (tray EMPTY)"),
    "cal":    ("cal_known",    [("mass_g", float, True), ("samples", int, False)],
               "cal <mass_g> [samples]  calibrate with known mass"),
    "rr":     ("read_raw",     [("samples", int, False)], "rr [samples]  raw avg/min/max + grams"),
    "setcal": ("set_cal",      [("scale", float, False), ("offset", int, False)],
               "setcal [scale] [offset]  set directly (use _ to skip scale)"),
    "gc":     ("get_cal",      [], "show calibration"),
    "cr":     ("cal_reset",    [], "scale back to default (not saved)"),
    "save":   ("save_cal",     [], "SAVE scale + offset to flash"),
}

# Full firmware command names also work as keys (e.g. `open`, `reset_fault`).
ALIASES = {
    "home": "h", "open": "o", "close": "c", "halt": "hl", "stop": "hl",
    "estop": "x", "reset_fault": "rf", "reset": "rf", "servo_reboot": "sr",
    "lock": "lk", "unlock": "ul", "status": "s", "diag": "d", "version": "v",
    "weigh": "w", "pos_query": "p", "pos": "p", "motor_diag": "md",
    "servo_status": "ss", "selftest": "st", "nop": "n", "tare": "t",
    "cal_known": "cal", "read_raw": "rr", "set_cal": "setcal", "get_cal": "gc",
    "cal_reset": "cr", "save_cal": "save", "raw_counts": "rc", "get_scale": "gs",
    "set_scale": "sc", "goto": "g", "jog": "j",
}

HELP_GROUPS = [
    ("Motion",      ["h", "o", "c", "hl"]),
    ("Bench jog (MOTION_BENCH_JOG=1 fw)", ["g", "j"]),
    ("Safety",      ["x", "rf", "sr", "lk", "ul"]),
    ("Query",       ["s", "d", "v", "w", "p", "md", "ss", "st", "n", "ping"]),
    ("Calibration", ["t", "cal", "rr", "setcal", "gc", "cr", "save"]),
    ("Legacy LC",   ["rc", "gs", "sc"]),
]

# Silenced unless toggled (they arrive continuously).
NOISY_EVENTS = {"heartbeat"}

RED, GREEN, YELLOW, CYAN, DIM, RESET = (
    "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[36m", "\x1b[2m", "\x1b[0m")


class Console:
    """Prints incoming lines without destroying the half-typed input line."""

    def __init__(self, color):
        self.color = color
        self.lock = threading.Lock()

    def paint(self, text, col):
        return f"{col}{text}{RESET}" if self.color else text

    def out(self, text, col=None):
        line = self.paint(text, col) if col else text
        with self.lock:
            if readline is not None and sys.stdin.isatty():
                buf = readline.get_line_buffer()
                sys.stdout.write("\r\x1b[K" + line + "\n" + PROMPT + buf)
            else:
                sys.stdout.write(line + "\n")
            sys.stdout.flush()


class Link:
    """TCP (auto-reconnect) or UDP link to the UART-Ethernet converter."""

    def __init__(self, host, port, udp, con):
        self.addr = (host, port)
        self.udp = udp
        self.con = con
        self.sock = None
        self.tx_lock = threading.Lock()
        self.connected = threading.Event()
        self.stop = threading.Event()

    def _open(self):
        if self.udp:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.5)
            s.connect(self.addr)          # fixes peer; recv() filters to it
        else:
            s = socket.create_connection(self.addr, timeout=3.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(0.5)
        return s

    def run_rx(self, on_line):
        buf = b""
        while not self.stop.is_set():
            if not self.connected.is_set():
                try:
                    self.sock = self._open()
                    self.connected.set()
                    buf = b""
                    self.con.out(f"[link] connected to {self.addr[0]}:{self.addr[1]}"
                                 f" ({'UDP' if self.udp else 'TCP'})", GREEN)
                except OSError as e:
                    self.con.out(f"[link] connect failed: {e} — retrying in 2 s", RED)
                    self.stop.wait(2.0)
                    continue
            try:
                data = self.sock.recv(4096)
                if not data and not self.udp:
                    raise ConnectionError("closed by peer")
            except socket.timeout:
                continue
            except OSError as e:
                self._drop(f"rx error: {e}")
                continue
            buf += data
            while True:
                i = min([k for k in (buf.find(b"\n"), buf.find(b"\r")) if k >= 0], default=-1)
                if i < 0:
                    break
                line, buf = buf[:i], buf[i + 1:]
                line = line.strip()
                if line:
                    on_line(line.decode("utf-8", "replace"))
            if len(buf) > 4096:       # no terminator for too long — discard
                buf = b""

    def _drop(self, why):
        if self.connected.is_set():
            self.con.out(f"[link] disconnected ({why}) — firmware will latch "
                         f"COMMS_LOST after ~45 s (3 missed pongs)", RED)
        self.connected.clear()
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass

    def send(self, obj):
        if not self.connected.is_set():
            return False
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        with self.tx_lock:
            try:
                self.sock.sendall(data) if not self.udp else self.sock.send(data)
                return True
            except OSError as e:
                self._drop(f"tx error: {e}")
                return False

    def close(self):
        self.stop.set()
        self._drop("quit")


class Client:
    def __init__(self, args):
        self.con = Console(color=not args.no_color and sys.stdout.isatty())
        self.link = Link(args.host, args.port, args.udp, self.con)
        self.ping_s = args.ping
        self.seq = 0
        self.seq_lock = threading.Lock()
        self.verbose = False          # show pongs + raw TX
        self.show_hb = False          # show heartbeat events
        self.last_pong = time.monotonic()
        self.pong_warned = False

    # -- TX ------------------------------------------------------------------
    def next_seq(self):
        with self.seq_lock:
            self.seq = (self.seq + 1) % 1_000_000
            return self.seq

    def send_cmd(self, cmd, fields=None, quiet=False):
        obj = {"cmd": cmd}
        if fields:
            obj.update(fields)
        obj["seq"] = self.next_seq()
        ok = self.link.send(obj)
        if not quiet:
            if ok:
                self.con.out("→ " + json.dumps(obj, separators=(",", ":")), CYAN)
            else:
                self.con.out("✗ not connected — command NOT sent", RED)
        return ok

    def keepalive(self):
        """Comms watchdog feed: ping every self.ping_s while connected."""
        while not self.link.stop.is_set():
            if not self.link.connected.is_set():
                # Poll quickly so the first ping goes out right after connect
                # (firmware blocks motion until its first host message).
                self.last_pong = time.monotonic()
                self.link.stop.wait(0.2)
                continue
            self.send_cmd("ping", quiet=not self.verbose)
            silent = time.monotonic() - self.last_pong
            if silent > 3 * self.ping_s + 2 and not self.pong_warned:
                self.pong_warned = True
                self.con.out(f"!! no pong for {silent:.0f} s — check converter/"
                             f"firmware (fault after 3 × 10 s without rx)", YELLOW)
            self.link.stop.wait(self.ping_s)

    # -- RX ------------------------------------------------------------------
    def on_line(self, line):
        try:
            msg = json.loads(line)
        except ValueError:
            self.con.out(f"   {line}", DIM)       # boot log / non-JSON text
            return
        if not isinstance(msg, dict):
            self.con.out(f"   {line}", DIM)
            return
        ev, resp = msg.get("event"), msg.get("resp")
        if msg.get("cmd") == "ping":
            # Device ping (FW 2.0.0+): answer with the same seq, or the tray
            # latches COMMS_LOST after 3 missed pongs (15 s each).
            obj = {"cmd": "pong"}
            if "seq" in msg:
                obj["seq"] = msg["seq"]
            ok = self.link.send(obj)
            self.last_pong = time.monotonic()
            if self.verbose:
                self.con.out("← " + line, DIM)
                self.con.out(("→ " if ok else "✗ ") + json.dumps(obj, separators=(",", ":")), DIM)
            return
        if resp == "pong" or ev == "pong":
            self.last_pong = time.monotonic()
            if self.pong_warned:
                self.pong_warned = False
                self.con.out("   pongs resumed", GREEN)
            if self.verbose:
                self.con.out("← " + line, DIM)
            return
        if ev in NOISY_EVENTS and not self.show_hb:
            return
        col = None
        if (resp == "nak" or ev in ("safe_state", "error")
                or msg.get("success") is False or msg.get("result") == "fault"):
            col = RED
        elif ev == "cmd_complete" and msg.get("result") == "ok":
            col = GREEN
        elif ev in ("cal_result", "tare_result", "reset_result") and msg.get("success"):
            col = GREEN
        elif ev in ("warning", "limit_hit", "state_change"):
            col = YELLOW
        self.con.out("← " + line, col)
        if ev == "safe_state" and msg.get("fault") == "COMMS_LOST":
            self.con.out("   COMMS_LOST latched — link is back? send `rf` to clear, "
                         "then `h` to home", YELLOW)

    # -- input ---------------------------------------------------------------
    def print_help(self):
        lines = ["", "Short keys (Enter to send):"]
        for title, keys in HELP_GROUPS:
            lines.append(f"  {title}:")
            for k in keys:
                lines.append(f"    {k:<7} {SHORTCUTS[k][2]}")
        lines += [
            "  Full names work too: open, close, home, halt, reset_fault, status, ...",
            "  Raw JSON: type a line starting with {   e.g. {\"cmd\":\"status\"}",
            "  Client:  vb  toggle verbose (pongs/pings)   hb  toggle heartbeats",
            "           help | ?    q  quit (link stops answering pings → fault in ~45 s)",
            f"  Keepalive: ping every {self.ping_s:g} s (firmware needs ≥1 msg / 10 s)",
            "",
        ]
        self.con.out("\n".join(lines))

    def build(self, tokens):
        alias, args = tokens[0].lower(), tokens[1:]
        alias = ALIASES.get(alias, alias)
        if alias not in SHORTCUTS:
            return None, f"unknown key '{alias}' — type help"
        cmd, spec, _ = SHORTCUTS[alias]
        if len(args) > len(spec):
            return None, f"too many arguments for '{alias}'"
        fields = {}
        for i, (name, typ, required) in enumerate(spec):
            if i >= len(args) or args[i] == "_":
                if required:
                    return None, f"'{alias}' needs <{name}>"
                continue
            try:
                fields[name] = typ(args[i])
            except ValueError:
                return None, f"<{name}> must be {typ.__name__}"
        if cmd == "set_cal" and not fields:
            return None, "setcal needs a scale and/or offset"
        return (cmd, fields), None

    def handle(self, line):
        line = line.strip()
        if not line:
            return True
        low = line.lower()
        if low in ("q", "quit", "exit"):
            return False
        if low in ("help", "?"):
            self.print_help()
            return True
        if low == "vb":
            self.verbose = not self.verbose
            self.con.out(f"   verbose {'ON' if self.verbose else 'OFF'}")
            return True
        if low == "hb":
            self.show_hb = not self.show_hb
            self.con.out(f"   heartbeats {'shown' if self.show_hb else 'hidden'}")
            return True
        if line.startswith("{"):
            try:
                obj = json.loads(line)
                if not isinstance(obj, dict) or "cmd" not in obj:
                    raise ValueError("needs a \"cmd\" field")
            except ValueError as e:
                self.con.out(f"   bad JSON: {e}", RED)
                return True
            obj.setdefault("seq", self.next_seq())
            ok = self.link.send(obj)
            self.con.out(("→ " + json.dumps(obj, separators=(",", ":"))) if ok
                         else "✗ not connected — command NOT sent", CYAN if ok else RED)
            return True
        built, err = self.build(line.split())
        if err:
            self.con.out("   " + err, RED)
            return True
        self.send_cmd(*built)
        return True

    def run(self):
        threading.Thread(target=self.link.run_rx, args=(self.on_line,), daemon=True).start()
        threading.Thread(target=self.keepalive, daemon=True).start()
        self.con.out("Sediment tray client — type `help`. Keepalive pings run in "
                     "the background.")
        try:
            while True:
                try:
                    line = input(PROMPT)
                except EOFError:
                    break
                if not self.handle(line):
                    break
        except KeyboardInterrupt:
            pass
        self.con.out("\n[client] quitting — firmware will latch COMMS_LOST in ~45 s "
                     "if nothing else is pinging it", YELLOW)
        self.link.close()


def main():
    ap = argparse.ArgumentParser(description="Sediment tray Ethernet client")
    ap.add_argument("--host", default="192.168.2.153",
                    help="UART-Ethernet converter IP (default 192.168.2.153)")
    ap.add_argument("--port", type=int, default=2000, help="converter TCP port (default 2000)")
    ap.add_argument("--udp", action="store_true", help="use UDP instead of TCP")
    ap.add_argument("--ping", type=float, default=2.0,
                    help="keepalive ping period in s (default 2, max 5)")
    ap.add_argument("--no-color", action="store_true", help="disable ANSI colours")
    args = ap.parse_args()
    if not 0.5 <= args.ping <= 5.0:
        ap.error("--ping must be 0.5..5 s (firmware window is 10 s)")
    Client(args).run()


if __name__ == "__main__":
    main()
