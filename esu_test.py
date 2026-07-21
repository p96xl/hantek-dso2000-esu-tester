#!/usr/bin/env python3
"""
ESU output test bench — Hantek DSO2C50 over SCPI/USBTMC (PyVISA).

Ch1 = voltage across load (1000:1 HV probe), Ch2 = current (Pearson 110A).
Pulls both traces via the DSO2000 PRIVate waveform export, computes RF power
three ways, plots the capture, and writes an HTML report (print to PDF).

Setup + method: vault note Projects/Hardware/ESU-Output-Test-Bench.md
Waveform protocol reverse-engineered from github.com/phmarek/hantek-dso2000.

Install:  pip install pyvisa pyvisa-py pyusb numpy matplotlib
Driver (offline): Zadig once -> bind scope USBTMC interface to WinUSB.
Run:      python esu_test.py --sn ELLMAN123 --mode "cut 50W" --load 500 --setpoint 50
          python esu_test.py --list        list VISA resources + IDN
          python esu_test.py --calcheck     capture CH1, print Vpp (verify probe-comp)
          python esu_test.py --demo         self-test, no scope
"""
import argparse, base64, datetime, os, struct, sys, warnings

warnings.filterwarnings('ignore', category=UserWarning, module=r'pyvisa_py')  # silence TCPIP/HiSLIP discovery noise

# libusb-1.0.dll lives next to this script or bundled in the exe (_MEIPASS).
_dll_dir = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
os.environ['PATH'] = _dll_dir + os.pathsep + os.environ.get('PATH', '')

# ---- bench constants — SET THESE FOR YOUR RIG -----------------------------
# Scope is the source of truth: set each scope channel's PROBE ratio to your real probe
# (CH1 = your HV probe's 10x/1000x; CH2 = 1x for the Pearson BNC). Then :CHANnel:SCALe?
# already reports true volts and the program does NOT re-scale voltage.
PROBE_RATIO = 1.0      # leave at 1 — the scope's own probe setting handles CH1 voltage scaling
# Current is unavoidable in software: the Pearson is 0.1 V/A and the scope only shows volts.
COIL_V_PER_A = 0.1     # Pearson 110A: 0.1 (1 MO input) or 0.05 (50 O input). Amps = CH2_volts / this. CH2 probe = 1x.
CODES_PER_DIV = 25.0   # ponytail: DSO2000 ADC = 25 codes/div (from ref impl). VERIFY via --calcheck.
# ---------------------------------------------------------------------------

BLOCK = 2000           # samples are interleaved in 2000-byte per-channel blocks
VDIV, HDIV = 8, 14     # DSO2000 grid: 8 vertical, 14 horizontal divisions


def _snap125(x, lo, hi, up=True):
    """Snap x to a 1-2-5 gear, clamped. up=True: smallest gear >= x (never overflow the range,
    for V/div). up=False: nearest gear (for timebase, so cycle count lands on target)."""
    import math
    if x <= 0:
        return lo
    dec = 10 ** math.floor(math.log10(x))
    gears = [m * dec for m in (1, 2, 5, 10)]
    if up:
        pick = next((g for g in gears if g >= x - 1e-15), gears[-1])
    else:
        pick = min(gears, key=lambda g: abs(g - x))
    return min(max(pick, lo), hi)


def autoscale(scope, targets=(1, 2), cycles=6):
    """Fill the ADC so a small signal stops reading as noise.
    The #1 cause of a 'noisy' trace here is a signal spanning only a few ADC codes.
    Per channel: measure peak, set V/div so peak sits at ~3 of 8 divisions (6-div span,
    headroom left). Then set the timebase from CH1's frequency to show ~`cycles` cycles."""
    import numpy as np, time
    chans, dt = capture(scope)                       # quick read under HRES + BWLimit
    for ch in targets:
        if ch not in chans:
            continue
        pk = float(np.max(np.abs(chans[ch]))) or 1e-3
        per_div = _snap125(pk / 3.0, 1e-3, 1e5)      # peak at ~3 div; snap up so it can't clip
        scope.write(f':CHANnel{ch}:OFFSet 0')
        scope.write(f':CHANnel{ch}:SCALe {per_div:g}')
    if 1 in chans:
        f = dom_freq(chans[1], dt)
        if f and np.isfinite(f):
            # nearest gear so the on-screen cycle count actually lands near `cycles`
            scope.write(f':TIMebase:SCALe {_snap125((cycles / f) / HDIV, 2e-9, 50, up=False):g}')
    time.sleep(0.3)                                  # let the new gears settle before the real capture


def connect(resource=None):
    import pyvisa
    try:
        rm = pyvisa.ResourceManager()          # NI-VISA if present
    except Exception:
        rm = pyvisa.ResourceManager('@py')     # else pure-python + libusb
    if resource is None:
        try:
            res = list(rm.list_resources('USB?*::INSTR'))   # filtered -> skips slow TCPIP/serial probing
        except Exception:
            res = []
        if not res:
            res = [r for r in rm.list_resources() if 'USB' in r and 'INSTR' in r]
        if not res:
            raise RuntimeError("No USBTMC instrument found. Saw: " + str(rm.list_resources()) +
                     "\n  1) Scope: Utility -> I/O -> USB Device = Computer/USBTMC.\n"
                     "  2) Bind the scope's USBTMC interface to WinUSB with Zadig (once).")
        resource = res[0]
    scope = rm.open_resource(resource)
    scope.timeout = 15000
    print("Connected:", scope.query('*IDN?').strip())
    return scope


def capture(scope, acq='HRESolution', count=64, bwlimit=True, fresh=True):
    """Return {phys_ch: np.array(volts)}, and seconds/sample.
    acq: NORMal | AVERage | PEAK | HRESolution (HRES = oversample-average, best for noise on a single frame).
    bwlimit: enable the channel 20MHz limit (cuts noise; Pearson caps at 20MHz anyway).
    fresh: RUN briefly so a new frame acquires under these settings before reading."""
    import numpy as np, time
    for ch in (1, 2):
        scope.write(f':CHANnel{ch}:BWLimit {1 if bwlimit else 0}')
    if acq:
        scope.write(f':ACQuire:TYPE {acq}')
        if acq.upper().startswith('AVER'):
            scope.write(f':ACQuire:COUNt {int(count)}')
    if fresh:
        # let a fresh frame build under the new settings. AVERage needs many triggers -> wait longer.
        scope.write(':RUN')
        time.sleep(1.5 if acq and acq.upper().startswith('AVER') else 0.4)
    scope.write(':STOP')                        # freeze for a coherent read
    buf = bytearray()
    total, meta, guard, retries = None, None, 0, 0
    while guard < 10000:
        scope.write('PRIVate:WAVeform:DATA:ALL?')
        raw = bytes(scope.read_raw())
        if len(raw) < 29 or raw[:2] != b'#9':   # 29-byte header (#9 + 9+9+9 digits). Short/empty = scope not ready.
            if len(buf) == 0 and retries < 20:  # only retry before real data has landed (re-query restarts at off=0)
                retries += 1; time.sleep(0.15); continue
            raise RuntimeError(f"bad/empty waveform packet (len {len(raw)}, {raw[:16]!r}) after {retries} retries — "
                               "check a signal is present and the scope is triggering/acquiring")
        total = int(raw[11:20]); off = int(raw[20:29])   # off = bytes already uploaded = this chunk's start
        if off == 0:
            meta = raw[29:128]                  # channel-enable etc. live in the first packet's header
        payload = raw[128:][:total - off]       # fw 1.0.8: 128-byte header on EVERY packet; clamp to bytes still needed
        if len(buf) < off:
            buf.extend(bytes(off - len(buf)))
        buf[off:off + len(payload)] = payload
        guard += 1
        if off + len(payload) >= total:
            break
    raw_all = bytes(buf[:total])

    # meta: channel-enable flags tell us how many channels are interleaved
    m = struct.unpack('cc 16x 7s7s7s7s cccc 9s 6s 9x 9s 6s 10x', meta)
    enables = m[6:10]                            # c1e..c4e
    enabled = [p for p, e in zip((1, 2, 3, 4), enables) if e not in (b'\x00', b'0')]
    n = len(enabled) or 1
    if len(raw_all) % n:
        raise RuntimeError(f"{len(raw_all)} samples not divisible by {n} enabled channels "
                           f"(enables={enables}) — check CH enable detection")
    data = np.frombuffer(raw_all, dtype=np.int8).astype(float)
    out = {}
    for ordinal, phys in enumerate(enabled):
        # gather this channel's 2000-sample blocks
        idx = np.concatenate([np.arange(i, min(i + BLOCK, len(data)))
                              for i in range(ordinal * BLOCK, len(data), BLOCK * n)]).astype(int)
        samp = data[idx]
        scale = float(scope.query(f':CHANnel{phys}:SCALe?'))
        offset = float(scope.query(f':CHANnel{phys}:OFFSet?'))
        out[phys] = samp / CODES_PER_DIV * scale - offset
    try:
        dt = float(scope.query(':WAVeform:XINCrement?'))
    except Exception:
        dt = 1.0 / float(scope.query(':ACQuire:SRATe?'))
    return out, dt


def dom_freq(x, dt):
    """Dominant frequency, CALCULATED from the trace — this firmware's :MEASure SCPI is dead,
    so the scope's own counter can't be read over USB. Method: measure the period from
    linearly-interpolated zero-crossings across the whole record (precise for a clean carrier),
    validated against a Hann-FFT peak so harmonic/noise miscounts fall back to the FFT estimate."""
    import numpy as np
    x = np.asarray(x, float)
    x = x - np.mean(x)
    N = len(x); span = N * dt
    sp = np.abs(np.fft.rfft(x * np.hanning(N))); sp[0] = 0
    k = int(np.argmax(sp))
    if k <= 0:
        return float('nan')
    if k < len(sp) - 1:                              # parabolic-interpolated FFT peak: robust ballpark
        y1, y2, y3 = sp[k - 1], sp[k], sp[k + 1]; d = y1 - 2 * y2 + y3
        f_fft = (k + (0.5 * (y1 - y3) / d if d else 0.0)) / span
    else:
        f_fft = k / span
    neg = x < 0                                      # refine via upward zero crossings (x[i]<0, x[i+1]>=0)
    ups = np.where(neg[:-1] & ~neg[1:])[0]
    if len(ups) >= 2:
        tc = ups + x[ups] / (x[ups] - x[ups + 1])    # sub-sample crossing index (linear interp)
        f_zc = (len(tc) - 1) / ((tc[-1] - tc[0]) * dt)
        if abs(f_zc - f_fft) < 1.5 / span:           # within ~1 FFT bin -> trust the precise one
            return f_zc
    return f_fft


def scope_meas(scope, item, ch=1):
    """Read a hardware measurement off the scope: :MEASure:CHANnel<n>:ITEM? <item> -> float, or None.
    fw 1.0.8 quirks (proven via scpi_scan.py): query the item DIRECTLY — do NOT :MEASure:ENABle,
    set :ITEM, or interleave :SYSTem:ERRor?; any of those desync the response buffer. VRMS is broken
    (returns the frequency) so only FREQuency/PERiod/VPP/VMAX/VAVG are usable. Occasional timeout -> None."""
    old = scope.timeout
    try:
        scope.timeout = 2000                    # short: a flaky query fails fast instead of hanging 15 s
        return float(scope.query(f':MEASure:CHANnel{ch}:ITEM? {item}').strip())
    except Exception:
        return None
    finally:
        scope.timeout = old


def metrics(V, I, R, dt, freq=None):
    """V, I already in real volts/amps. Returns the measurement + 3 power estimates.
    freq: if given (e.g. the scope's own counter), used verbatim; else computed from the trace."""
    import numpy as np
    Vrms = float(np.sqrt(np.mean(V**2))); Irms = float(np.sqrt(np.mean(I**2)))
    Vpk = float(np.max(np.abs(V)))
    return {
        'Vrms': Vrms, 'Vpp': float(np.ptp(V)), 'Vpeak': Vpk,
        'Irms': Irms, 'Ipeak': float(np.max(np.abs(I))),
        'Freq_Hz': freq if freq is not None else dom_freq(V, dt),
        'CrestFactor': Vpk / Vrms if Vrms else float('nan'),
        'P_from_V (Vrms^2/R)': Vrms**2 / R,
        'P_from_I (Irms^2*R)': Irms**2 * R,
        'P_from_VxI (mean v*i)': float(np.mean(V * I)),
    }


def plot(t, V, I, path, smooth=0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    Id = I
    if smooth and smooth > 1:                    # DISPLAY-ONLY moving average — does not touch the RMS/power numbers
        k = np.ones(int(smooth)) / int(smooth)
        Id = np.convolve(I, k, mode='same')
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    a1.plot(t * 1e6, V, lw=0.8); a1.set_ylabel('Voltage (V)'); a1.grid(alpha=.3)
    a2.plot(t * 1e6, Id, lw=0.8, color='tab:red'); a2.set_ylabel('Current (A)')
    a2.set_xlabel('time (µs)'); a2.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)
    return path


def report(info, mets, img_path, out_path):
    b64 = base64.b64encode(open(img_path, 'rb').read()).decode()
    rows = "".join(f"<tr><td>{k}</td><td>{v:.4g}</td></tr>" for k, v in mets.items())
    hdr = "".join(f"<tr><td><b>{k}</b></td><td>{v}</td></tr>" for k, v in info.items())
    html = f"""<!doctype html><meta charset=utf-8>
<style>body{{font:14px sans-serif;margin:2em}}table{{border-collapse:collapse;margin:1em 0}}
td{{border:1px solid #ccc;padding:4px 10px}}h1{{font-size:20px}}img{{max-width:720px;border:1px solid #999}}</style>
<h1>ESU Output Test Report</h1>
<table>{hdr}</table>
<h2>Measurements</h2><table>{rows}</table>
<h2>Capture</h2><img src="data:image/png;base64,{b64}">
"""
    open(out_path, 'w', encoding='utf-8').write(html)   # utf-8: report contains non-ASCII (⚠, µ, ²)
    print("Report ->", out_path, "(open in browser, Ctrl+P -> Save as PDF)")


def generate(resource, sn, mode, setpoint, load, out, smooth=0, acq='HRESolution', count=64,
             turns=1, auto=True, cycles=6):
    """Capture, compute, write report. Returns (metrics dict, report path). Shared by CLI + GUI.
    turns: # of passes of the ESU wire through the Pearson coil (Pearson reads amp-TURNS,
           so N passes gives N x the signal off the noise floor; we divide amps back by N).
    auto:  autoscale the scope V/div + timebase to fill the ADC before capturing."""
    import numpy as np
    scope = connect(resource)
    try:
        if auto:
            autoscale(scope, cycles=cycles)
        chans, dt = capture(scope, acq=acq, count=count)
        if 1 not in chans or 2 not in chans:
            raise RuntimeError(f"Need CH1 (voltage) and CH2 (current) enabled; captured {sorted(chans)}")
        V = chans[1] * PROBE_RATIO              # real volts across load
        I = chans[2] / COIL_V_PER_A / turns     # real amps: undo the coil V/A and the N turns
        fs = scope_meas(scope, 'FREQuency')     # scope's own counter (matches the display); None if it times out
        fs = fs if fs and 1e3 < fs < 1e8 else None
        mets = metrics(V, I, load, dt, freq=fs)  # metrics from RAW current — smoothing is display-only
        img = plot(np.arange(len(V)) * dt, V, I, out.replace('.html', '.png'), smooth)
        # resolution guard: warn if a channel's signal barely spans the ADC (coarse V/div = garbage numbers)
        codes = {ch: np.ptp(chans[ch]) / (float(scope.query(f':CHANnel{ch}:SCALe?')) / CODES_PER_DIV)
                 for ch in (1, 2)}
        warn = "; ".join(f"CH{ch} spans only {c:.0f} ADC codes — reduce CH{ch} V/div"
                         for ch, c in codes.items() if c < 10)
        if warn:
            print("  !! LOW RESOLUTION: " + warn)
        mets['CH2 ADC codes (pp)'] = codes[2]
        info = {
            'Unit S/N': sn, 'Mode': mode, 'Front-panel setpoint (W)': setpoint,
            'Load (ohm)': load, 'Pearson coil turns': turns,
            'Date': datetime.datetime.now().isoformat(timespec='seconds'),
            'Instrument': scope.query('*IDN?').strip(), 'Samples/ch': len(V), 'dt (s)': dt,
        }
        if warn:
            info['⚠ WARNING'] = "LOW ADC RESOLUTION — " + warn + " on the scope for a valid measurement"
        report(info, mets, img, out)
        return mets, out
    finally:
        scope.close()


def run(args):
    mets, _ = generate(args.resource, args.sn, args.mode, args.setpoint, args.load,
                       args.out, args.smooth, args.acq, args.count, args.turns,
                       not args.no_autoscale, args.cycles)
    print("  P (Vrms^2/R) = %.1f W | P (Irms^2*R) = %.1f W | P (mean v*i) = %.1f W"
          % (mets['P_from_V (Vrms^2/R)'], mets['P_from_I (Irms^2*R)'], mets['P_from_VxI (mean v*i)']))


def session(args):
    """Power-sweep mode: connect + autoscale ONCE, then capture on each Enter with the
    scope held open — skips the per-test reconnect + re-autoscale (the slow parts).
    Each capture prints the 3-way power and appends a row to <out>_sweep.csv."""
    import csv, os
    scope = connect(args.resource)
    try:
        if not args.no_autoscale:
            print("Set the ESU to your HIGHEST sweep power first, then autoscaling so nothing clips at the top...")
            autoscale(scope, cycles=args.cycles)
        csvpath = args.out.replace('.html', '') + '_sweep.csv'
        new = not os.path.exists(csvpath)
        f = open(csvpath, 'a', newline='')
        w = csv.writer(f)
        if new:
            w.writerow(['setpoint_W', 'P_Vrms2R', 'P_Irms2R', 'P_meanVI', 'Vrms', 'Irms', 'Freq_Hz'])
        print("\nSweep. Dial a power, type the watts + Enter to capture. 'a'=re-autoscale, 'q'=quit.")
        while True:
            s = input("setpoint W> ").strip()
            if s.lower() in ('q', 'quit', 'exit'):
                break
            if s.lower() in ('a', 'auto'):
                autoscale(scope, cycles=args.cycles); print("  re-autoscaled."); continue
            chans, dt = capture(scope, acq=args.acq, count=args.count)
            if 1 not in chans or 2 not in chans:
                print("  !! need CH1 (voltage) + CH2 (current) enabled"); continue
            V = chans[1] * PROBE_RATIO
            I = chans[2] / COIL_V_PER_A / args.turns
            fs = scope_meas(scope, 'FREQuency')
            fs = fs if fs and 1e3 < fs < 1e8 else None
            m = metrics(V, I, args.load, dt, freq=fs)
            print("  P: Vrms2/R=%.1f  Irms2*R=%.1f  v*i=%.1f W  |  Vrms=%.1f Irms=%.3f f=%.0fHz" % (
                m['P_from_V (Vrms^2/R)'], m['P_from_I (Irms^2*R)'], m['P_from_VxI (mean v*i)'],
                m['Vrms'], m['Irms'], m['Freq_Hz']))
            w.writerow([s, m['P_from_V (Vrms^2/R)'], m['P_from_I (Irms^2*R)'],
                        m['P_from_VxI (mean v*i)'], m['Vrms'], m['Irms'], m['Freq_Hz']])
            f.flush()
        f.close()
        print("Sweep saved ->", csvpath)
    finally:
        scope.close()


def live(args):
    """Rolling live waveform: re-capture + redraw until you close the window.
    NOT true streaming — the DSO2000 has no streaming SCPI, only whole-frame block reads,
    so refresh is ~1-3 Hz at shallow memory. ponytail: poll-redraw is the ceiling this
    firmware allows; a faster path would need streaming SCPI the scope doesn't have."""
    import matplotlib.pyplot as plt
    import numpy as np
    scope = connect(args.resource)
    try:
        if not args.no_autoscale:
            autoscale(scope, cycles=args.cycles)
        plt.ion()
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        print("Live view — close the window to stop.")
        while plt.fignum_exists(fig.number):
            chans, dt = capture(scope, acq=args.acq, count=args.count)
            if 1 not in chans or 2 not in chans:
                print("need CH1 (voltage) + CH2 (current) enabled"); break
            V = chans[1] * PROBE_RATIO
            I = chans[2] / COIL_V_PER_A / args.turns
            t = np.arange(len(V)) * dt * 1e6
            fs = scope_meas(scope, 'FREQuency')
            fs = fs if fs and 1e3 < fs < 1e8 else None
            m = metrics(V, I, args.load, dt, freq=fs)
            a1.clear(); a2.clear()
            a1.plot(t, V, lw=.8); a1.set_ylabel('Voltage (V)'); a1.grid(alpha=.3)
            a1.set_title("v·i=%.1f W  |  Vrms=%.1f  Irms=%.3f  f=%.0f Hz" % (
                m['P_from_VxI (mean v*i)'], m['Vrms'], m['Irms'], m['Freq_Hz']))
            a2.plot(t, I, lw=.8, color='tab:red'); a2.set_ylabel('Current (A)')
            a2.set_xlabel('time (µs)'); a2.grid(alpha=.3)
            plt.pause(0.05)
    finally:
        scope.close()


def gui():
    """Dead-simple tech GUI: fill 4 fields, click, get a report. Stdlib tkinter, ships in the exe."""
    import tkinter as tk
    from tkinter import messagebox
    import webbrowser, traceback
    root = tk.Tk(); root.title("ESU Output Test")
    root.geometry("440x310")
    form = [("Unit S/N", "sn", ""), ("Mode", "mode", "cut 50W"),
            ("Setpoint (W)", "setpoint", ""), ("Load (ohm)", "load", "500"),
            ("Coil turns", "turns", "1"),           # passes of ESU wire through the Pearson: more turns = cleaner current
            ("Current smoothing", "smooth", "0")]   # display-only; 0=off, try 5-15 if the current plot looks noisy
    ent = {}
    for i, (label, key, default) in enumerate(form):
        tk.Label(root, text=label).grid(row=i, column=0, sticky="e", padx=8, pady=5)
        e = tk.Entry(root, width=30); e.insert(0, default)
        e.grid(row=i, column=1, padx=8, pady=5); ent[key] = e
    status = tk.StringVar(value="Ready. Plug in scope, set CH1=voltage / CH2=current.")

    def busy(msg):
        status.set(msg); root.update_idletasks()   # ponytail: sync capture (<1s at 4k pts); thread it only if deep memory feels laggy

    def do_test():
        busy("Testing connection...")
        try:
            s = connect(None); idn = s.query('*IDN?').strip(); s.close()
            status.set("Connected: " + idn)
        except Exception as e:
            status.set("No scope: " + str(e))

    def do_capture():
        busy("Capturing + building report...")
        try:
            sn = ent['sn'].get().strip() or 'NA'
            out = f"esu_report_{sn}.html".replace(' ', '_')
            mets, path = generate(None, sn, ent['mode'].get(), ent['setpoint'].get(),
                                  float(ent['load'].get()), out, int(ent['smooth'].get() or 0),
                                  turns=int(ent['turns'].get() or 1))
            status.set("P: Vrms²/R=%.1fW · Irms²·R=%.1fW · v·i=%.1fW · freq=%.0fHz" % (
                mets['P_from_V (Vrms^2/R)'], mets['P_from_I (Irms^2*R)'],
                mets['P_from_VxI (mean v*i)'], mets['Freq_Hz']))
            webbrowser.open(os.path.abspath(path))
        except Exception as e:
            status.set("Error: " + str(e))
            messagebox.showerror("Error", traceback.format_exc())

    r = len(form)
    tk.Button(root, text="Test Connection", command=do_test).grid(row=r, column=0, padx=8, pady=10)
    tk.Button(root, text="Capture & Report", command=do_capture, height=2,
              font=("", 10, "bold")).grid(row=r, column=1, padx=8, pady=10, sticky="we")
    tk.Label(root, textvariable=status, wraplength=420, justify="left",
             fg="#036").grid(row=r + 1, column=0, columnspan=2, padx=8, pady=8)
    root.mainloop()


def calcheck(resource=None):
    """Diagnostic: confirm HRES/BWLimit actually applied, and report per-channel noise stats."""
    import numpy as np
    scope = connect(resource)

    def q(cmd):
        try:
            return scope.query(cmd).strip()
        except Exception as e:
            return f'<{type(e).__name__}>'

    chans, dt = capture(scope)                        # sets HRES + BWLimit, then reads
    print("acquisition settings the scope reports back:")
    print("  :ACQuire:TYPE?     =", q(':ACQuire:TYPE?'), " (want HRES)")
    print("  :ACQuire:COUNt?    =", q(':ACQuire:COUNt?'))
    print("  :ACQuire:POINts?   =", q(':ACQuire:POINts?'), " (small = fast USB transfer; deep memory is slow)")
    print("  :ACQuire:SRATe?    =", q(':ACQuire:SRATe?'))
    for ch in (1, 2):
        print(f"  CH{ch}: BWLimit={q(f':CHANnel{ch}:BWLimit?')} SCALe={q(f':CHANnel{ch}:SCALe?')} "
              f"OFFSet={q(f':CHANnel{ch}:OFFSet?')}")
    print("captured (scope volts, before probe ratio / coil):")
    for ch, v in chans.items():
        print(f"  CH{ch}: mean={np.mean(v):+.4f}V  std(noise)={np.std(v):.4f}V  "
              f"Vpp={np.ptp(v):.4f}V  ~codes_pp={np.ptp(v) / (float(q(f':CHANnel{ch}:SCALe?')) / CODES_PER_DIV):.1f}")
    fs = scope_meas(scope, 'FREQuency')
    import numpy as np
    print(f"frequency: scope :MEASure counter = {fs}  vs computed from CH1 = {dom_freq(chans[1], dt):.0f} Hz"
          if fs else f"frequency: scope counter timed out; computed from CH1 = {dom_freq(chans[1], dt):.0f} Hz")
    print(f"dt={dt}s/sample. mean = DC offset, std = noise. If :ACQuire:TYPE? is NOT HRES, the firmware rejected it.")
    scope.close()


def list_resources():
    import pyvisa
    try:
        rm = pyvisa.ResourceManager()
    except Exception:
        rm = pyvisa.ResourceManager('@py')
    res = rm.list_resources()
    if not res:
        print("Nothing found. Scope in Computer/USBTMC mode? WinUSB bound via Zadig?")
        return
    for r in res:
        try:
            with rm.open_resource(r) as inst:
                inst.timeout = 3000
                print(f"{r}  ->  {inst.query('*IDN?').strip()}")
        except Exception as e:
            print(f"{r}  ->  (no IDN: {e})")


def probe(resource=None):
    """Dump the multi-packet structure of PRIVate:WAVeform:DATA:ALL? to diagnose reassembly."""
    scope = connect(resource)
    scope.timeout = 10000
    print("ACQuire:POINts? =", scope.query(':ACQuire:POINts?').strip())
    scope.write(':STOP')
    print("-- packet dump --")
    total = None; k = 0
    while k < 30:
        scope.write('PRIVate:WAVeform:DATA:ALL?')
        raw = bytes(scope.read_raw())
        if raw[:2] != b'#9':
            print(f"pkt{k}: BAD header {raw[:16]!r}"); break
        plen = int(raw[2:11]); total = int(raw[11:20]); off = int(raw[20:29])
        payload_len = len(raw) - (128 if off == 0 else 29)
        print(f"pkt{k}: rawlen={len(raw)} plen={plen} total={total} off={off} "
              f"payloadlen={payload_len} end={off + payload_len} head14={raw[:14]!r}")
        k += 1
        if off + payload_len >= total:
            break
    scope.write(':RUN')
    scope.close()


def demo():
    """Self-check: 100 Vrms sine across 500 ohm = 20 W; all three power methods agree."""
    import numpy as np
    t = np.linspace(0, 1e-3, 20000, endpoint=False)
    V = 100 * np.sqrt(2) * np.sin(2 * np.pi * 4000 * t)   # 100 Vrms, 4 kHz
    I = V / 500.0                                          # resistive 500 ohm
    m = metrics(V, I, 500, t[1] - t[0])
    for key in ('P_from_V (Vrms^2/R)', 'P_from_I (Irms^2*R)', 'P_from_VxI (mean v*i)'):
        assert abs(m[key] - 20) < 0.1, (key, m[key])
    assert abs(m['Freq_Hz'] - 4000) < 20, m['Freq_Hz']
    assert abs(m['Vrms'] - 100) < 0.1
    # _snap125 must round UP to the next 1-2-5 gear so the chosen range never clips the signal
    assert _snap125(0.1, 1e-3, 1e5) == 0.1 and _snap125(0.11, 1e-3, 1e5) == 0.2
    assert _snap125(3, 1e-3, 1e5) == 5 and _snap125(0.03, 1e-3, 1e5) == 0.05
    assert _snap125(1e-6, 1e-3, 1e5) == 1e-3 and _snap125(1e9, 1e-3, 1e5) == 1e5  # clamped
    assert _snap125(0.76, 1e-9, 50, up=False) == 1 and _snap125(0.6, 1e-9, 50, up=False) == 0.5  # nearest gear
    # frequency at a FRACTIONAL FFT bin (like 471 kHz in a 32 µs window) — interp must beat the ~31 kHz bin grid
    t2 = np.linspace(0, 32e-6, 4000, endpoint=False)
    f_est = dom_freq(np.sin(2 * np.pi * 471000 * t2), t2[1] - t2[0])
    assert abs(f_est - 471000) < 3000, f_est   # raw bin-picker would land on 500 kHz (29 kHz off)
    assert metrics(V, I, 500, t[1] - t[0], freq=472000)['Freq_Hz'] == 472000  # scope-supplied freq used verbatim
    print("demo OK — 20 W by all three methods, freq 4 kHz + fractional-bin 471 kHz, Vrms 100 V, snap125")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--gui', action='store_true', help='launch the tech GUI (also the default on double-click)')
    ap.add_argument('--session', action='store_true', help='power-sweep: connect+autoscale once, capture on each Enter, log to CSV (fast for many setpoints)')
    ap.add_argument('--live', action='store_true', help='rolling live waveform window (poll+redraw; close window to stop)')
    ap.add_argument('--demo', action='store_true', help='run self-test, no scope')
    ap.add_argument('--list', action='store_true', help='list VISA resources + IDN')
    ap.add_argument('--calcheck', action='store_true', help='capture + print Vpp to verify scaling')
    ap.add_argument('--probe', action='store_true', help='dump waveform packet structure (diagnostic)')
    ap.add_argument('--resource', help='VISA resource (default: autodetect USB)')
    ap.add_argument('--sn', default='?', help='ESU unit serial number')
    ap.add_argument('--mode', default='?', help='e.g. "cut 50W" / "coag"')
    ap.add_argument('--setpoint', default='?', help='front-panel watts')
    ap.add_argument('--load', type=float, default=500, help='load resistance (ohm)')
    ap.add_argument('--turns', type=int, default=1, help='passes of the ESU wire through the Pearson coil (N turns = Nx signal, amps divided back by N)')
    ap.add_argument('--no-autoscale', action='store_true', help='skip auto V/div + timebase; use the scope as-is')
    ap.add_argument('--cycles', type=int, default=6, help='approx # of waveform cycles to show on screen (autoscale timebase)')
    ap.add_argument('--smooth', type=int, default=0, help='display-only current smoothing window (samples); 0=off')
    ap.add_argument('--acq', default='HRESolution', help='acquisition: NORMal|AVERage|PEAK|HRESolution (HRES cuts noise)')
    ap.add_argument('--count', type=int, default=64, help='averages when --acq AVERage')
    ap.add_argument('--out', default='esu_report.html')
    a = ap.parse_args()
    if a.gui or len(sys.argv) == 1: gui()   # no args (double-click) -> GUI
    elif a.session: session(a)
    elif a.live: live(a)
    elif a.demo: demo()
    elif a.list: list_resources()
    elif a.calcheck: calcheck(a.resource)
    elif a.probe: probe(a.resource)
    else: run(a)
