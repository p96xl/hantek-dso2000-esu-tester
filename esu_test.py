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


def autoscale(scope, targets=(1, 2), cycles=6, set_timebase=True):
    """Fill the ADC so a small signal stops reading as noise.
    The #1 cause of a 'noisy' trace here is a signal spanning only a few ADC codes.
    Per channel: measure peak, set V/div so peak sits at ~3 of 8 divisions (6-div span,
    headroom left). Then set the timebase from CH1's frequency to show ~`cycles` cycles."""
    import numpy as np, time
    # ponytail: ESU carriers are 0.3-4 MHz. Start fast so dom_freq can't lock onto an alias --
    # at a slow timebase the scope drops to ~125 kSa/s, dom_freq returns the beat (e.g. 309 Hz),
    # and we'd set an even slower timebase from it. That loop latches and never recovers.
    scope.write(':TIMebase:SCALe 1e-6')              # 14 us window: ~56 cyc @4 MHz, ~4 cyc @300 kHz
    time.sleep(0.2)
    chans, dt = capture(scope)                       # quick read under HRES + BWLimit
    for ch in targets:
        if ch not in chans:
            continue
        pk = float(np.max(np.abs(chans[ch]))) or 1e-3
        per_div = _snap125(pk / 3.0, 1e-3, 1e5)      # peak at ~3 div; snap up so it can't clip
        scope.write(f':CHANnel{ch}:OFFSet 0')
        scope.write(f':CHANnel{ch}:SCALe {per_div:g}')
    if set_timebase and 1 in chans:
        f = dom_freq(chans[1], dt)
        if f and np.isfinite(f):
            if not 1e5 <= f <= 1e7:                  # outside any ESU band -> we're seeing an alias
                print(f"  WARNING: measured {f:,.0f} Hz -- not an ESU carrier (0.3-4 MHz). "
                      "Aliased or no RF present; power numbers will be wrong.")
            # nearest gear so the on-screen cycle count actually lands near `cycles`
            # hi=2e-5 caps at 20 us/div -- keeps the sample rate above the carrier no matter what f says
            scope.write(f':TIMebase:SCALe {_snap125((cycles / f) / HDIV, 2e-9, 2e-5, up=False):g}')
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


def burst_spread(V, chunks=8):
    """RMS scatter across sub-windows of one capture. ~0 for a CW mode (cut).
    Large when the capture straddles a modulation envelope (blend/coag/fulg) -- which means
    the window is too SHORT to average the envelope, so Vrms and power are whatever the
    trigger happened to land on. Not a signal fault; a windowing fault."""
    import numpy as np
    n = len(V) // chunks
    if n < 16:
        return 0.0
    r = np.array([np.sqrt(np.mean(V[i * n:(i + 1) * n] ** 2)) for i in range(chunks)])
    return float(r.std() / r.mean()) if r.mean() else 0.0


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
        # |phase| between V and I via power factor. Magnitude only -- read lead/lag off the scope.
        # Should be ~0 on a resistive load; a big angle means reactance (stray C) is in the I path.
        'Phase_deg': float(np.degrees(np.arccos(
            np.clip(np.mean(V * I) / (Vrms * Irms), -1, 1)))) if Vrms and Irms else float('nan'),
        'BurstSpread': burst_spread(V),
    }


def warn_modulated(m):
    """Print a warning if this capture can't be trusted as an average-power reading.
    Modulated ESU modes (blend/coag/fulg) need a window spanning whole envelope periods."""
    if m['BurstSpread'] > 0.25:
        print(f"  WARNING: envelope varies {100 * m['BurstSpread']:.0f}% across this capture -- the "
              "window is shorter than the modulation period.\n"
              "           Vrms and power are whatever the trigger landed on, NOT average power. "
              "See README 'Modulated modes'.")
        return True
    return False


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
             turns=1, auto=True, cycles=6, vmult=1.0):
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
        V = chans[1] * PROBE_RATIO * vmult     # real volts across load (vmult = load-tap ratio, e.g. 3 if probing 1 of 3 equal series Rs)
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
            'Load (ohm)': load, 'Pearson coil turns': turns, 'Voltage tap x': vmult,
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
                       not args.no_autoscale, args.cycles, args.vmult)
    print("  P (Vrms^2/R) = %.1f W | P (Irms^2*R) = %.1f W | P (mean v*i) = %.1f W  [phase %.0f deg]"
          % (mets['P_from_V (Vrms^2/R)'], mets['P_from_I (Irms^2*R)'],
             mets['P_from_VxI (mean v*i)'], mets['Phase_deg']))
    # mean(v*i) never uses R, so Vrms^2/P_real is an R-independent cross-check on the whole chain.
    # It scales as Vrms/Irms, so a gap is EITHER the typed load OR the V/I scaling (probe
    # attenuation, coil V/A, termination) -- it can't tell you which. ~10% is normal at 4 MHz.
    p_real = mets['P_from_VxI (mean v*i)']
    if p_real > 0:
        r_impl = mets['Vrms'] ** 2 / p_real
        err = abs(r_impl - args.load) / args.load
        flag = "  <-- >20%: check load AND probe/coil scaling" if err > 0.20 else ""
        print("  implied Z = %.0f ohm vs --load %g (%+.0f%%)%s" % (r_impl, args.load, 100 * (r_impl / args.load - 1), flag))
    warn_modulated(mets)
    if mets['P_from_VxI (mean v*i)'] < 0:
        # A passive load cannot source power. Only a polarity error can do this.
        print("  WARNING: real power is NEGATIVE -- a passive load cannot source power, so one "
              "probe is backwards. Flip the voltage probe or the current coil and re-run.")


def envelope_power(scope, args, repeats=3):
    """Average power for MODULATED modes (blend / coag / fulg) — and a valid cross-check on CW.

    Method: ONE LONG WINDOW, plain `mean(v*i)`.
    V and I are sampled simultaneously, so v(t)*i(t) is correct at every instant no matter how
    badly the 4 MHz carrier is aliased. Average over a window spanning many modulation periods
    and the carrier phase averages out along with the envelope. No peak detect, no /2, no phase
    correction — those were all artifacts of trying to reconstruct the envelope.

    Verified in simulation to 0.0-1.6% on cut(CW) / cutcoag / hemo / fulg at 50 and 125 kSa/s.

    The one failure mode is COHERENT sampling: if the sample rate divides the carrier exactly,
    every sample lands at the same phase and the answer can be anything. Guarded by repeating
    the capture and requiring agreement — that is the only honest check.

    NOTE: `:ACQuire:TYPE PEAK` is NOT honoured by fw 1.0.8 (verified 2026-07-27 — it returns
    ordinary decimated samples). An earlier version assumed peak detect worked and read exactly
    HALF on CW as a result. Acquisition type and timebase are both read back below."""
    import numpy as np, time

    # 1. FAST timebase: autoscale V/div (amplitude moves a lot across a sweep) and confirm a
    #    real carrier is present. Retry -- a fast window can land in a gap between bursts.
    carrier, mf = float('nan'), None
    for attempt in range(6):
        scope.write(f':TIMebase:SCALe {_snap125(1e-6, 2e-9, 50, up=False):g}')
        time.sleep(0.2)
        if not args.no_autoscale:
            autoscale(scope, cycles=args.cycles, set_timebase=False)   # V/div only
        chans, dt = capture(scope, acq='NORMal', count=args.count)
        if 1 not in chans or 2 not in chans:
            raise RuntimeError("need CH1 (voltage) + CH2 (current) enabled")
        mf = metrics(chans[1] * PROBE_RATIO * args.vmult,
                     chans[2] / COIL_V_PER_A / args.turns, args.load, dt)
        if 1e5 <= mf['Freq_Hz'] <= 1e7:
            carrier = mf['Freq_Hz']
            break
        print(f"  (retry {attempt + 1}: fast window saw {mf['Freq_Hz']:,.0f} Hz -- between bursts)")
    else:
        raise RuntimeError("never caught an in-burst carrier in 6 tries -- is the ESU keying?")

    # 2. LONG window. NORMal, never HRES -- HRES boxcar-averages the carrier to nothing.
    scope.write(f':TIMebase:SCALe {_snap125(args.envwin / HDIV, 2e-9, 50):g}')
    time.sleep(0.3)
    got = float(scope.query(':TIMebase:SCALe?'))
    if got * HDIV < args.envwin * 0.5:
        raise RuntimeError(f"timebase did not take: asked {args.envwin / HDIV:g} s/div, scope is at "
                           f"{got:g} ({got * HDIV * 1e3:.1f} ms window). Set it by hand.")

    ms = []
    for _ in range(repeats):                    # repeats ARE the validation (coherent-sampling guard)
        chans, dte = capture(scope, acq='NORMal', count=args.count)
        ms.append(metrics(chans[1] * PROBE_RATIO * args.vmult,
                          chans[2] / COIL_V_PER_A / args.turns, args.load, dte))
    Ps = np.array([x['P_from_VxI (mean v*i)'] for x in ms])
    spread = float(Ps.std() / Ps.mean()) if Ps.mean() else float('nan')
    m = dict(ms[0]); m['P_from_VxI (mean v*i)'] = float(Ps.mean()); m['Freq_Hz'] = carrier
    print(f"  carrier {carrier:,.0f} Hz, {got * HDIV * 1e3:.0f} ms window | envelope: "
          + " | ".join(f"{p:.1f}" for p in Ps) + f"  -> {Ps.mean():.1f} W (spread {100 * spread:.1f}%)")
    if spread > 0.05:
        print("  WARNING: repeats disagree >5% -- raise --envwin; this reading is not trustworthy")
    # Coherent sampling gives a STABLE wrong answer, so `spread` can't see it. The fast capture
    # resolves the carrier, so its Vpeak is the true peak; phase-uniform slow sampling over
    # thousands of samples must still land near it. Much lower = samples stuck at one phase.
    # (A low Vrms/Vpeak ratio does NOT work as the test -- modulated modes read low legitimately.)
    hit = ms[-1]['Vpeak'] / mf['Vpeak'] if mf['Vpeak'] else float('nan')
    if hit < 0.8:
        print(f"  WARNING: long-window Vpeak is only {hit:.0%} of the carrier-resolved Vpeak -- "
              "samples are missing the peaks (sample rate near a divisor of the carrier). "
              "Nudge --envwin to shift the sample rate, then re-run.")
    return m, spread, carrier


def envelope(args):
    """--envelope: measure a modulated mode. Run it on CUT first to validate the method."""
    scope = connect(args.resource)
    try:
        m, _, _ = envelope_power(scope, args)
        print(f"  Vrms={m['Vrms']:.1f} Irms={m['Irms']:.3f} phase={m['Phase_deg']:.0f}deg")
    finally:
        scope.close()


def session_name(parts):
    """--session ellman 1234 cut  ->  'ellman_1234_cut'. Unquoted words are joined so the
    natural shell form works; anything path-unsafe becomes '_'."""
    import re
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', '_'.join(parts)).strip('_') or 'session'


def upsert(rows, index, key, row, named):
    """Named session: a repeat (mode,setting) REPLACES the earlier row in place, so the sweep
    keeps its original order and --compare sees one row per setting. Bare session: append
    everything. Returns the row that was replaced, else None."""
    if named and key in index:
        old = rows[index[key]]
        rows[index[key]] = row
        return old
    if named:
        index[key] = len(rows)
    rows.append(row)
    return None


def verdict(got, expected, tol_pct):
    """PASS iff got is inside expected +/- tol_pct. Returns (ok, lo, hi, pct_dev)."""
    lo, hi = expected * (1 - tol_pct / 100), expected * (1 + tol_pct / 100)
    dev = 100 * (got / expected - 1) if expected else float('nan')
    return lo <= got <= hi, lo, hi, dev


def compare(args):
    """Score a saved session against an OEM reference table and chart it.

    Reference CSV columns: mode,setting,load_ohm,expected_W,tol_pct (+ optional model).
    Point it at any machine's table with --ref; the default is only a convenience.

    The OEM table is a CURVE, so expected_W is INTERPOLATED between tabulated settings --
    sweep at whatever granularity you like. Settings outside the table's range are reported
    as skipped, never silently dropped, and never extrapolated.

    Measured column is P_meanVI: mean(v*i) uses neither the typed load nor assumes the load
    is resistive, so it's the only one worth grading against."""
    import csv, os, glob, numpy as np, matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # --ref takes a bare machine name (resolved in refs/) or an explicit path.
    refdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'refs')
    ref = args.ref
    if not os.path.exists(ref):
        cand = os.path.join(refdir, ref if ref.endswith('.csv') else ref + '.csv')
        if os.path.exists(cand):
            ref = cand
        else:
            have = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(refdir, '*.csv')))
            raise SystemExit(f"No reference '{args.ref}'. Machines in refs/: "
                             + (", ".join(have) if have else "(none yet)")
                             + "\n  Add one as refs/<machine>.csv — see refs/README.md")
    args.ref = ref
    curves, models = {}, set()
    with open(args.ref, newline='') as f:
        for r in csv.DictReader(f):
            curves.setdefault(r['mode'].strip().lower(), []).append(
                (float(r['setting']), float(r['expected_W']), float(r['tol_pct']), float(r['load_ohm'])))
            if r.get('model'):
                models.add(r['model'].strip())
    for c in curves.values():
        c.sort()

    def ref_at(mode, setting):
        """Interpolate (expected_W, tol_pct, load_ohm) on the mode's curve. None if off the ends."""
        c = curves.get(mode)
        if not c:
            return None
        xs = [p[0] for p in c]
        if not xs[0] <= setting <= xs[-1]:
            return None
        return tuple(float(np.interp(setting, xs, [p[i] for p in c])) for i in (1, 2, 3))

    rows, skipped = [], []
    with open(args.compare, newline='') as f:
        for r in csv.DictReader(f):
            try:
                setting = float(r['setting'])
            except (ValueError, KeyError):
                continue
            # pre-existing sweeps have no mode column -- fall back to --mode
            mode = (r.get('mode') or args.mode or '').strip().lower()
            hit = ref_at(mode, setting)
            if hit is None:
                skipped.append((mode, setting)); continue
            exp, tol, load = hit
            got = float(r['P_meanVI'])
            ok, lo, hi, dev = verdict(got, exp, tol)
            rows.append(dict(mode=mode, setting=setting, got=got, exp=exp, lo=lo, hi=hi,
                             dev=dev, ok=ok, load=load, meas_load=float(r.get('load_ohm') or load),
                             method=(r.get('method') or '').strip(),
                             spread=float(r.get('spread_pct') or 0)))

    if skipped:
        modes = sorted({m for m, _ in skipped})
        print(f"  SKIPPED {len(skipped)} row(s) not covered by {args.ref}: modes={modes}, "
              f"settings {min(s for _, s in skipped):g}-{max(s for _, s in skipped):g}")
        print(f"  reference covers: " + ", ".join(
            f"{m} {c[0][0]:g}-{c[-1][0]:g}" for m, c in sorted(curves.items())))
    if not rows:
        print(f"No rows in {args.compare} could be graded against {args.ref}.")
        return

    # name the reference in the report -- grading a machine against another model's table
    # is the one failure mode here that produces a confident, wrong PASS.
    print(f"\n{args.compare}  vs  {args.ref}" + (f"  [{', '.join(sorted(models))}]" if models else ""))
    print(f"{'mode':<10}{'set':>5}{'meas W':>9}{'spec W':>9}{'window':>17}{'dev':>8}   result")
    for r in sorted(rows, key=lambda x: (x['mode'], x['setting'])):
        print(f"{r['mode']:<10}{r['setting']:>5.0f}{r['got']:>9.1f}{r['exp']:>9.1f}"
              f"{r['lo']:>8.1f}-{r['hi']:<8.1f}{r['dev']:>+7.1f}%   {'PASS' if r['ok'] else 'FAIL'}"
              + (f"  [{r['method']}]" if r['method'] else "")
              + (f"  <-- repeats scattered {r['spread']:.0f}%, NOT trustworthy" if r['spread'] > 5 else ""))
        if abs(r['meas_load'] - r['load']) / r['load'] > 0.05:
            print(f"{'':<10}  ^ load was {r['meas_load']:.0f} ohm, spec assumes {r['load']:.0f} ohm")
    npass = sum(r['ok'] for r in rows)
    print(f"\n{npass}/{len(rows)} within tolerance")

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for i, mode in enumerate(sorted({r['mode'] for r in rows})):
        c = f'C{i}'
        pts = curves[mode]
        xs = [p[0] for p in pts]
        ax.plot(xs, [p[1] for p in pts], '-', color=c, lw=1.2, label=f'{mode} spec')
        ax.fill_between(xs, [p[1] * (1 - p[2] / 100) for p in pts],
                        [p[1] * (1 + p[2] / 100) for p in pts], color=c, alpha=0.15,
                        label=f'{mode} ±{pts[0][2]:.0f}%')
        mine = [r for r in rows if r['mode'] == mode]
        for ok, mark in ((True, 'o'), (False, 'X')):
            sel = [r for r in mine if bool(r['ok']) == ok]
            if sel:
                ax.scatter([r['setting'] for r in sel], [r['got'] for r in sel], marker=mark,
                           s=90, color=c, edgecolor='k', zorder=5,
                           label=f'{mode} measured ({"pass" if ok else "FAIL"})')
    ax.set_xlabel('Digital setting'); ax.set_ylabel('Power (W)')
    ax.set_title(f'{os.path.basename(args.compare)} — measured vs OEM spec')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    out = args.compare.replace('.csv', '') + '_vs_spec.png'
    fig.tight_layout(); fig.savefig(out, dpi=130)
    print("Chart ->", out)


def session(args):
    """Power-sweep mode: connect + autoscale ONCE, then capture on each Enter with the
    scope held open — skips the per-test reconnect + re-autoscale (the slow parts).
    Each capture prints the 3-way power and writes a row to <name>_sweep.csv.

    NAMED session -> a record: re-capturing a (mode, setting) REPLACES the earlier row in
    place, so a redo overwrites the bad reading instead of leaving both for --compare to
    trip over. BARE --session -> a scratch log: every capture is kept, nothing overwritten."""
    import csv, os, time, numpy as np
    named, env = bool(args.session), False
    scope = connect(args.resource)
    try:
        if not args.no_autoscale:
            print("Set the ESU to your HIGHEST sweep power first, then autoscaling so nothing clips at the top...")
            autoscale(scope, cycles=args.cycles)
        csvpath = session_name(args.session) + '_sweep.csv'
        HDR = ['mode', 'setting', 'load_ohm', 'method', 'P_Vrms2R', 'P_Irms2R', 'P_meanVI',
               'Vrms', 'Irms', 'Freq_Hz', 'Phase_deg', 'spread_pct']
        rows, index = [], {}          # index maps (mode,setting)->row position; named runs only
        if os.path.exists(csvpath):   # resume: reread so a re-run keeps overwriting, not duplicating
            with open(csvpath, newline='') as fh:
                for r in csv.DictReader(fh):
                    rows.append([r.get(c, '') for c in HDR])   # tolerates older column sets
                    if named:
                        index[(r.get('mode', ''), r.get('setting', ''))] = len(rows) - 1
            print(f"Resuming {csvpath} ({len(rows)} existing row(s))")

        def save():
            with open(csvpath, 'w', newline='') as fh:
                wr = csv.writer(fh); wr.writerow(HDR); wr.writerows(rows)

        print(f"\nLogging to {csvpath} (mode={args.mode}, load={args.load:g} ohm)")
        print("  redo policy: " + ("re-capturing a setting OVERWRITES it" if named
                                   else "append-only (bare --session), every capture kept"))
        print("Sweep. Type the DIAL SETTING + Enter to capture.")
        print("  'm <mode>'=switch mode  'l <ohms>'=change load  'e'=envelope mode (modulated"
              " modes)  'a'=re-autoscale  'q'=quit")
        while True:
            s = input(f"[{args.mode}] setting> ").strip()
            if s.lower() in ('q', 'quit', 'exit'):
                break
            if s.lower() in ('a', 'auto'):
                autoscale(scope, cycles=args.cycles); print("  re-autoscaled."); continue
            if s.lower().startswith('m ') or s.lower() == 'm':
                # one session per machine, not per mode -- rows carry their own mode column
                new = s[1:].strip()
                if not new:
                    print(f"  mode is '{args.mode}'. Use 'm hemo' to switch."); continue
                args.mode = new
                print(f"  mode -> {args.mode}  (remember to change the load if this mode needs a "
                      f"different one; currently {args.load:g} ohm, set with 'l <ohms>')")
                continue
            if s.lower() in ('e', 'env'):
                env = not env
                if not env:
                    autoscale(scope, cycles=args.cycles)      # back to a carrier-length window
                print(f"  envelope mode {'ON — autoscale + long window on every capture' if env else 'OFF'}")
                continue
            if s.lower().startswith('l '):
                try:
                    args.load = float(s[1:].strip())
                except ValueError:
                    print("  need a number, e.g. 'l 200'"); continue
                print(f"  load -> {args.load:g} ohm"); continue
            if env:
                m, spread, _ = envelope_power(scope, args)
            else:
                spread = 0.0
                chans, dt = capture(scope, acq=args.acq, count=args.count)
                if 1 not in chans or 2 not in chans:
                    print("  !! need CH1 (voltage) + CH2 (current) enabled"); continue
                fs = scope_meas(scope, 'FREQuency')
                m = metrics(chans[1] * PROBE_RATIO * args.vmult,
                            chans[2] / COIL_V_PER_A / args.turns, args.load, dt,
                            freq=fs if fs and 1e3 < fs < 1e8 else None)
            print("  P: Vrms2/R=%.1f  Irms2*R=%.1f  v*i=%.1f W  |  Vrms=%.1f Irms=%.3f f=%.0fHz ph=%.0fdeg" % (
                m['P_from_V (Vrms^2/R)'], m['P_from_I (Irms^2*R)'], m['P_from_VxI (mean v*i)'],
                m['Vrms'], m['Irms'], m['Freq_Hz'], m['Phase_deg']))
            if not env:
                warn_modulated(m)               # in env mode the wide window is the fix already
            row = [args.mode, s, args.load, 'envelope' if env else 'direct',
                   m['P_from_V (Vrms^2/R)'], m['P_from_I (Irms^2*R)'],
                   m['P_from_VxI (mean v*i)'], m['Vrms'], m['Irms'], m['Freq_Hz'],
                   m['Phase_deg'], round(100 * spread, 1)]
            old = upsert(rows, index, (args.mode, s), row, named)
            if old is not None:
                print(f"  overwrote earlier capture at {args.mode} {s} "
                      f"({float(old[6]):.1f} W -> {m['P_from_VxI (mean v*i)']:.1f} W)")
            save()                          # rewrite after every capture -- a crash loses nothing
        print(f"Sweep saved -> {csvpath} ({len(rows)} row(s))")
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
            V = chans[1] * PROBE_RATIO * args.vmult
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
    # --vmult scales V, so Vrms^2/R scales by vmult^2 (probe 1/3 the load -> vmult 3 -> 9x)
    assert abs(metrics(V * 3, I, 500, t[1] - t[0])['P_from_V (Vrms^2/R)']
               - 9 * metrics(V, I, 500, t[1] - t[0])['P_from_V (Vrms^2/R)']) < 1e-6
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
    # --session name joining, and the pass/fail band the comparison grades on
    assert session_name(['ellman', '1234', 'cut']) == 'ellman_1234_cut'
    assert session_name([]) == 'session' and session_name(['a/b c']) == 'a_b_c'
    assert verdict(76.4, 88, 20)[0] and verdict(105.1, 120, 20)[0]     # both real CUT points pass
    assert not verdict(69, 88, 20)[0] and not verdict(107, 88, 20)[0]  # just outside either edge
    assert verdict(70.4, 88, 20)[0] and verdict(105.6, 88, 20)[0]      # edges are inclusive
    # named session: a redo replaces in place and keeps sweep order; bare session keeps both
    rows, idx = [], {}
    for s, w in (('100', 106.1), ('70', 12.3), ('50', 74.6), ('70', 89.6)):
        upsert(rows, idx, ('cut', s), ['cut', s, w], True)
    assert [r[1] for r in rows] == ['100', '70', '50'], rows      # no duplicate, order preserved
    assert rows[1][2] == 89.6                                     # the redo won
    rows, idx = [], {}
    for s, w in (('70', 12.3), ('70', 89.6)):
        upsert(rows, idx, ('cut', s), ['cut', s, w], False)
    assert len(rows) == 2                                         # bare --session keeps both
    # long-window mean(v*i) recovers TRUE average power on modulated modes AND on CW, from
    # ordinary decimated samples -- no peak detect (fw 1.0.8 ignores :ACQuire:TYPE PEAK).
    import math
    fc, R, dtc = 3999533.0, 500.0, 1 / 400e6          # off-integer carrier, like the real unit
    tc = np.arange(0, 80e-3, dtc)
    Cs = math.tan(math.radians(13)) / (2 * math.pi * fc * R)
    for envf in (np.ones_like(tc),                                       # cut (CW)
                 np.abs(np.sin(2 * np.pi * 120 * tc)),                   # blend, full-wave
                 np.clip(np.sin(2 * np.pi * 60 * tc), 0, None)):         # coag, half-wave
        Vm = 300 * np.sqrt(2) * envf * np.sin(2 * np.pi * fc * tc)
        Im = Vm / R + Cs * np.gradient(Vm, dtc)
        dec = int(round((1 / 50e3) / dtc))            # slow window -> ~50 kSa/s, carrier aliased
        assert abs(np.mean(Vm[::dec] * Im[::dec]) / np.mean(Vm * Im) - 1) < 0.03
    # and the failure the old peak-detect math produced on CW: exactly HALF
    Vm = 300 * np.sqrt(2) * np.sin(2 * np.pi * fc * tc); Im = Vm / R
    dec = int(round((1 / 50e3) / dtc))
    half = np.mean(np.abs(Vm[::dec]) * np.abs(Im[::dec])) / 2
    assert abs(half / np.mean(Vm * Im) - 0.5) < 0.02, half   # the 50% underread, reproduced
    print("demo OK — 20 W by all three methods, freq 4 kHz + fractional-bin 471 kHz, Vrms 100 V, snap125, session/verdict")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--gui', action='store_true', help='launch the tech GUI (also the default on double-click)')
    ap.add_argument('--session', nargs='*', metavar='NAME', help='power-sweep: connect+autoscale once, capture on each Enter, log to <NAME>_sweep.csv. Words are joined: --session ellman 1234 cut -> ellman_1234_cut_sweep.csv')
    ap.add_argument('--envelope', action='store_true', help='measure a MODULATED mode (blend/coag/fulg) via peak-detect envelope averaging; run on cut first to validate')
    ap.add_argument('--envwin', type=float, default=50e-3, metavar='SEC', help='envelope averaging window in seconds (default 0.05 = 3 cycles of a 60 Hz envelope)')
    ap.add_argument('--compare', metavar='SWEEP.CSV', help='score a saved session against --ref and chart it (no scope needed)')
    ap.add_argument('--ref', default='ellman-surgitron-4.0', metavar='MACHINE', help='OEM reference table: a name resolved in refs/<name>.csv, or an explicit path. Columns: model,mode,setting,load_ohm,expected_W,tol_pct')
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
    ap.add_argument('--vmult', type=float, default=1.0, help='voltage tap multiplier = load-divider ratio when the probe is across only PART of the load (e.g. 3 when probing 1 of 3 equal series resistors). Corrects V so all 3 power methods agree.')
    ap.add_argument('--no-autoscale', action='store_true', help='skip auto V/div + timebase; use the scope as-is')
    ap.add_argument('--cycles', type=int, default=6, help='approx # of waveform cycles to show on screen (autoscale timebase)')
    ap.add_argument('--smooth', type=int, default=0, help='display-only current smoothing window (samples); 0=off')
    ap.add_argument('--acq', default='HRESolution', help='acquisition: NORMal|AVERage|PEAK|HRESolution (HRES cuts noise)')
    ap.add_argument('--count', type=int, default=64, help='averages when --acq AVERage')
    ap.add_argument('--out', default='esu_report.html')
    a = ap.parse_args()
    if a.gui or len(sys.argv) == 1: gui()   # no args (double-click) -> GUI
    elif a.compare: compare(a)
    elif a.envelope: envelope(a)
    elif a.session is not None: session(a)   # [] when --session given bare -> still a session
    elif a.live: live(a)
    elif a.demo: demo()
    elif a.list: list_resources()
    elif a.calcheck: calcheck(a.resource)
    elif a.probe: probe(a.resource)
    else: run(a)
