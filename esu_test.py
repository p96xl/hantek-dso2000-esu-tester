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
CODES_PER_DIV = 25.0   # DSO2000 ADC = 25 codes/div. VERIFIED on our DSO2C50 fw 1.0.8 (2026-09-09)
                       # by stepping :CHANnel1:OFFSet one division at a time: +25,+25,+26,-24,-25 codes.
ADC_RAIL = 127         # signed-int8 saturation. MEASURED, and it is NOT the screen edge: the ADC rails
                       # at +/-128 codes = +/-5.12 divisions, while the screen is 8 div (+/-100 codes).
                       # So a trace drawn clipped at the top of the screen still carries VALID data out
                       # to 1.28x the screen edge; only past +/-127 does it truly saturate and read low.
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


def clip_state(v, scale, offset):
    """Where a channel's samples sit against the ADC rail, in raw codes.
    Returns (peak_code, fraction_of_samples_at_the_rail). Undoes the volts conversion in
    capture(): samp = (volts + offset)/scale*CODES_PER_DIV."""
    import numpy as np
    codes = np.abs((np.asarray(v, float) + offset) / scale * CODES_PER_DIV)
    return float(codes.max()), float(np.mean(codes >= ADC_RAIL))


def clip_warn(scope, chans, targets=(1, 2)):
    """Print the clipping verdict per channel; return the set that is genuinely saturated.

    Two DIFFERENT states get confused by eye, which is why this prints both:
      * past the screen edge (>4 div) but under the rail -> looks clipped, reads CORRECTLY.
        This is the one that causes needless re-reads.
      * at the +/-127 rail -> flat-topped samples, Vrms and power read LOW. Real, silent error.
    """
    bad = set()
    for ch in targets:
        if ch not in chans:
            continue
        scale = float(scope.query(f':CHANnel{ch}:SCALe?'))
        offset = float(scope.query(f':CHANnel{ch}:OFFSet?'))
        top, railed = clip_state(chans[ch], scale, offset)
        if railed > 0.005:
            bad.add(ch)
            print(f"  !! CH{ch} SATURATED — {railed:.1%} of samples at the +/-{ADC_RAIL}-code ADC rail. "
                  f"Vrms and power READ LOW. Raise CH{ch} V/div (now {scale:g} V/div).")
        elif top > 4 * CODES_PER_DIV:
            print(f"  (CH{ch} runs {top / (4 * CODES_PER_DIV):.2f}x past the SCREEN edge but is not "
                  f"saturated — peak {top:.0f} of {ADC_RAIL} codes. This reading is VALID.)")
    return bad


CREST_MAX = 10.0       # fulg runs ~4-6; 10 leaves room and still rejects a 4000 V reply by 400x


def fire_test(item, thr, min_hz=100.0):
    """(fired, released) predicates for one detector reading. None always means 'do not know'
    — never 'idle' — so a dropped query cannot be mistaken for the pedal coming up.

    FREQuency: an idle channel measures no frequency at all, so the test is existence, not
    level. min_hz is only there to reject a stale VPP reply that leaked into the queue —
    those are volts (0.002 .. a few) and a keyed reading is kilohertz, so the two never
    overlap. A real aliased carrier can land below min_hz occasionally; the detector polls
    about once a second through a multi-second burst, so it catches the next one."""
    if str(item).upper().startswith('FREQ'):
        return (lambda v: v is not None and v > min_hz,
                lambda v: v == 0.0)
    return (lambda v: v is not None and v > thr,
            lambda v: v is not None and v <= thr * 0.5)


def vpp_bound(max_watts, load, turns):
    """Largest Vpp the current channel can physically show — from the RIG, not from the scope.

    Ipeak = crest x sqrt(P/R), and the Pearson turns that into COIL_V_PER_A x turns volts per
    amp, so Vpp = 2 x crest x sqrt(P/R) x COIL_V_PER_A x turns. Every term is known from the
    command line. That is the whole point: the fire-detector needs a sanity bound on a
    :MEASure reply, and asking the scope for one is what desyncs the reply buffer."""
    import math
    return 2 * CREST_MAX * math.sqrt(max_watts / load) * COIL_V_PER_A * turns


def screen_pp(scale):
    """Peak-to-peak the 8-division screen spans at this V/div. The fire-detector's natural
    unit: "has the trace left the baseline" is a question about the range in use, not about
    volts, so a threshold stated as a fraction of this stays right when the range changes."""
    return VDIV * scale


def vpp_ceiling(scale):
    """Largest peak-to-peak a channel can physically report at this V/div: the ADC rails at
    +/-127 codes and there are 25 codes/div. Any :MEASure VPP above it is a bad reply, not a
    big signal — fw 1.0.8's measurement path is documented in refs/ as returning a FREQUENCY
    for VRMS (16670 / 25000 / 5556 against real volts) and as desyncing its reply buffer when
    other queries are interleaved with it."""
    return 2 * ADC_RAIL / CODES_PER_DIV * scale        # = 1.27 screens


def predict_vdiv(watts, load, turns, vmult, crest=3.5):
    """(CH1, CH2) V/div computed from the wattage we EXPECT — no scope query, no iteration.

    This is the whole answer to "why is ranging slow". Measure-step-remeasure costs 2-5 s with
    the pedal DOWN, and it is solving for a number we already know: the wizard is testing a
    named mode at a named setting whose expected output is sitting in the profile. Set the
    ranges while the pedal is still UP and the keyed window is just the capture.

    Peak lands at 3 of 8 divisions, same target autoscale aims for, which leaves 2.7x to the
    screen edge and 3.4x to the ADC rail. Being wrong is cheap and self-correcting: clip_warn
    catches a railed frame and re-ranges the old way for that one burst.

    `crest` is peak/rms — cut (CW) is 1.41, coag and fulg run 4-6. The first burst of a mode
    uses --crest-hint; every burst after uses the crest that mode actually measured."""
    import math
    if not watts or watts <= 0 or load <= 0:
        return None
    vpk = crest * math.sqrt(watts * load) / max(vmult, 1e-9)      # scope sees V_true / vmult
    ipk = crest * math.sqrt(watts / load) * COIL_V_PER_A * turns  # Pearson volts, N turns
    return _snap125(vpk / 3.0, 1e-3, 1e4), _snap125(ipk / 3.0, 1e-3, 1e4)


def expected_w(bands, mode, setting):
    """Midpoint of the OEM band for this mode+setting, or 0.0 if the table has no such row."""
    try:
        want = float(setting)
    except (TypeError, ValueError):
        return 0.0
    for s_, lo, hi in bands.get(str(mode).strip().lower(), []):
        if s_ == want:
            return (lo + hi) / 2
    return 0.0


def range_by_vpp(scope, targets=(1, 2), tries=2):
    """Set V/div from the scope's own :MEASure:VPP instead of transferring a whole frame.

    autoscale() pulls a full 4K waveform per pass (~2.3 s each) purely to compute a peak.
    VPP is one query (~1 s) and is the ONE :MEASure item proven alive on fw 1.0.8 — it is
    what the fire-detector already runs on. Read on the LONG envelope window it is also more
    correct than autoscale's fast pass, which ranges off whatever point of the envelope a
    14 us slice happened to land on.

    Every second of this is a second the ESU is keyed into the load, so it matters more than
    the wall clock: a hot heatsink changes the number being measured."""
    import time
    for _ in range(tries):
        moved = False
        # ALL the :MEASure calls, then a device clear, THEN the ordinary queries. Never
        # alternating: that is what desyncs this firmware's reply buffer, and the stale answer
        # comes back as whatever was asked before it. A run reported CH2 at 4000 V/div — the
        # answer to an earlier :ACQuire:POINts? — beside a 0.002 V Vpp on the same channel.
        vpps = {c: scope_meas(scope, 'VPP', c) for c in targets}
        drain(scope)
        scales = {c: float(scope.query(f':CHANnel{c}:SCALe?')) for c in targets}
        for ch in targets:
            vpp, scale = vpps[ch], scales[ch]
            full = vpp_ceiling(scale)                     # peak-to-peak the ADC can actually hold
            if not vpp or vpp != vpp or vpp > full * 1.05:   # None/NaN/impossible -> leave it alone
                continue
            # Railed: VPP *is* the rail, so the true amplitude is unknown — step up and re-look,
            # same reasoning as autoscale. Otherwise put the p-p across ~6 of 8 divisions.
            want = (_snap125(scale * 2.5, 1e-3, 1e5) if vpp >= full * 0.98
                    else _snap125(vpp / 6.0, 1e-3, 1e5))
            if want != scale:
                scope.write(f':CHANnel{ch}:OFFSet 0')
                scope.write(f':CHANnel{ch}:SCALe {want:g}')
                moved = True
        if not moved:
            return
        time.sleep(0.3)


def autoscale(scope, targets=(1, 2), cycles=6, set_timebase=True, passes=4):
    """Fill the ADC so a small signal stops reading as noise.
    The #1 cause of a 'noisy' trace here is a signal spanning only a few ADC codes.
    Per channel: measure peak, set V/div so peak sits at ~3 of 8 divisions (6-div span,
    headroom left). Then set the timebase from CH1's frequency to show ~`cycles` cycles.

    ITERATES, because one pass cannot recover from a railed start: when the trace is already
    clipped the measured peak IS the rail, so the true amplitude is unknown and all we can do
    is step the range up and look again.

    set_timebase=False now leaves the timebase completely alone. It used to force 1 us/div
    regardless -- which is what broke envelope mode: V/div got chosen from a 14 us slice of an
    ~8 ms envelope, the slice landed on the envelope flank, the peak read far too low, and the
    real burst peaks then railed on the long window."""
    import numpy as np, time
    if set_timebase:
        # ponytail: ESU carriers are 0.3-4 MHz. Start fast so dom_freq can't lock onto an alias --
        # at a slow timebase the scope drops to ~125 kSa/s, dom_freq returns the beat (e.g. 309 Hz),
        # and we'd set an even slower timebase from it. That loop latches and never recovers.
        scope.write(':TIMebase:SCALe 1e-6')          # 14 us window: ~56 cyc @4 MHz, ~4 cyc @300 kHz
        time.sleep(0.2)
    chans, dt = capture(scope)                       # quick read under HRES + BWLimit
    for _ in range(passes):
        moved = False
        for ch in targets:
            if ch not in chans:
                continue
            scale = float(scope.query(f':CHANnel{ch}:SCALe?'))
            offset = float(scope.query(f':CHANnel{ch}:OFFSet?'))
            _, railed = clip_state(chans[ch], scale, offset)
            if railed > 0.005:
                per_div = _snap125(scale * 2.5, 1e-3, 1e5)   # railed: true peak unknown, step up and re-look
            else:
                pk = float(np.max(np.abs(chans[ch]))) or 1e-3
                per_div = _snap125(pk / 3.0, 1e-3, 1e5)      # peak at ~3 div; snap up so it can't clip
            if per_div != scale:
                scope.write(f':CHANnel{ch}:OFFSet 0')
                scope.write(f':CHANnel{ch}:SCALe {per_div:g}')
                moved = True
        if not moved:
            break
        time.sleep(0.3)
        chans, dt = capture(scope)                   # re-look under the new range
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


def looks_numeric(text):
    """True if this reply is a bare number — i.e. a stray measurement, not an identity string.
    Beware `any(c.isalpha())`: 1.316e+03 contains a letter and sailed through that test."""
    try:
        float(text)
        return True
    except (TypeError, ValueError):
        return False


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
    drain(scope, deep=True)                    # a previous run's orphaned replies outlive it
    idn = ''
    for attempt in range(4):                   # the queue can hold more than one orphan
        idn = scope.query('*IDN?').strip()
        if not looks_numeric(idn):
            break
        # "1.316e+03" is a stray FREQuency reply, not an identity. An earlier version tested
        # for letters and passed it straight through, because 'e' is a letter.
        print(f"  (stale reply in the scope's queue: *IDN? answered {idn!r} — clearing)")
        drain(scope)
    print("Connected:", idn)
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
    # Every text value we need is read BEFORE the binary transfer. A `.query()` issued AFTER
    # a waveform block read decodes whatever bytes are left in the output queue and dies with
    # "'ascii' codec can't decode byte 0x80 in position 94" — 0x80 is a waveform sample, not
    # text. Text first, then drain, then binary: the two never share the queue.
    cal = {}
    for c in (1, 2):
        try:
            cal[c] = (float(scope.query(f':CHANnel{c}:SCALe?')),
                      float(scope.query(f':CHANnel{c}:OFFSet?')))
        except Exception:
            pass
    drain(scope)                                # nothing text-shaped pending before we go binary
    buf = bytearray()
    total, meta, guard, retries, restarts = None, None, 0, 0, 0
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
        elif meta is None:
            # Joined a transfer already in flight: the channel-enable header rides ONLY on the
            # off==0 packet, so this frame cannot be decoded. Happens when a previous capture
            # was abandoned part-way and the scope is still delivering it. Restart the frame
            # instead of unpacking None, which surfaced as "a bytes-like object is required".
            if restarts < 3:
                restarts += 1
                print(f"  (joined a waveform transfer at offset {off} — restarting the frame)")
                drain(scope, deep=True)
                buf, total = bytearray(), None
                time.sleep(0.2)
                continue
            raise RuntimeError(
                "never saw the start of a waveform frame in 3 tries — the scope is still "
                "delivering an abandoned transfer. Unplug/replug the USB cable, or power-cycle "
                "the scope, and run again.")
        payload = raw[128:][:total - off]       # fw 1.0.8: 128-byte header on EVERY packet; clamp to bytes still needed
        if len(buf) < off:
            buf.extend(bytes(off - len(buf)))
        buf[off:off + len(payload)] = payload
        guard += 1
        if off + len(payload) >= total:
            break
    if meta is None or total is None:
        raise RuntimeError("waveform transfer ended without a header packet — nothing to decode")
    raw_all = bytes(buf[:total])
    drain(scope)                                # binary residue gone BEFORE any text query
    # dt stays here, after the transfer, because that is where it was proven to read correctly
    # — hoisting it above the block read would change what the scope reports it against.
    try:
        dt = float(scope.query(':WAVeform:XINCrement?'))
    except Exception:
        dt = 1.0 / float(scope.query(':ACQuire:SRATe?'))

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
        if phys not in cal:
            raise RuntimeError(f"CH{phys} is enabled on the scope but its scale/offset could not "
                               f"be read — samples would be scaled by a guess")
        scale, offset = cal[phys]
        out[phys] = samp / CODES_PER_DIV * scale - offset
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


def drain(scope, deep=False):
    """Throw away anything still sitting in the scope's output queue (USBTMC Device Clear).

    deep=True also reads off and discards whatever arrives next. Device Clear does NOT always
    abort a block transfer the scope has already begun, so a process that died mid-waveform
    leaves the rest of that frame queued for whoever connects next — which is how a fresh run
    joins a transfer at offset 128000 and finds no header. Costs one short timeout, so it is
    used at connect and on a mid-stream restart, not on the per-poll path.

    THE root cause of every desync in this tool. A :MEASure query that times out is not
    cancelled -- the scope still produces that reply, it just arrives after we stopped
    listening, and the NEXT query reads it instead of its own answer. One timeout poisons
    every reading after it, and the queue SURVIVES THE PROCESS: a fresh run's `*IDN?` came
    back as `2.000e-03` (the previous run's orphaned CH2 Vpp) and `1.190e+03` (an orphaned
    FREQuency). That is where the 4000 V/div and the 12 kV arm threshold came from."""
    try:
        scope.clear()
    except Exception:
        pass
    if not deep:
        return
    old = getattr(scope, 'timeout', None)
    try:
        scope.timeout = 150
        for _ in range(8):                      # bounded: a stuck scope must not hang the run
            if not scope.read_raw():
                break
    except Exception:
        pass                                    # a timeout here is the goal: the queue is empty
    finally:
        if old is not None:
            try: scope.timeout = old
            except Exception: pass


def scope_meas(scope, item, ch=1):
    """Read a hardware measurement off the scope: :MEASure:CHANnel<n>:ITEM? <item> -> float, or None.
    fw 1.0.8 quirks (proven via scpi_scan.py): query the item DIRECTLY — do NOT :MEASure:ENABle,
    set :ITEM, or interleave :SYSTem:ERRor?; any of those desync the response buffer. VRMS is broken
    (returns the frequency) so only FREQuency/PERiod/VPP/VMAX/VAVG are usable. Occasional timeout -> None."""
    old = scope.timeout
    src = 'MATH' if str(ch).upper() == 'MATH' else f'CHANnel{ch}'   # MATH is a source like a channel
    try:
        scope.timeout = 3000                    # short: a flaky query fails fast instead of hanging 15 s
        return float(scope.query(f':MEASure:{src}:ITEM? {item}').strip())
    except Exception:
        # The reply to THIS query is still coming. Drop it now or the next query gets it.
        drain(scope)
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
        railed = clip_warn(scope, chans)
        if railed and auto:                     # one re-range + re-capture, then report honestly
            autoscale(scope, cycles=cycles)
            chans, dt = capture(scope, acq=acq, count=count)
            railed = clip_warn(scope, chans)
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
        if railed:
            warn = (warn + "; " if warn else "") + \
                   "SATURATED on " + ", ".join(f"CH{c}" for c in sorted(railed)) + " — power reads LOW"
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
    carrier, mf, fresh_mf = float('nan'), None, False
    cached = getattr(args, '_carrier', None)
    if cached and not args.recarrier:
        # The carrier is a property of the MACHINE, not of the dial setting, so re-hunting it
        # at every sweep point costs ~7 s to re-learn a number that cannot have changed.
        carrier, mf = cached
        print(f"  carrier {carrier:,.0f} Hz (cached — --recarrier to re-measure)")
        # Cache the CARRIER, never the RANGE. V/div is a property of the DIAL SETTING and
        # moves ~20x across a 0 -> 10 walk, so reusing the last setting's range is exactly how
        # the top of a sweep ends up clipped. Skipping this is also what let the deliberate
        # post-burst headroom accumulate instead of being zoomed back in.
        # Range STRAIGHT ON THE LONG WINDOW with VPP: ~2 s of queries instead of ~6 s of
        # full-frame autoscale passes, and it sees the real envelope peaks rather than a
        # 14 us slice of them. The carrier hunt is the only thing the cache skips.
        if args.no_autoscale or getattr(args, '_ranged', False):
            print("  (ranges pre-set with the pedal up — no ranging in the keyed window)")
        else:
            scope.write(f':TIMebase:SCALe {_snap125(args.envwin / HDIV, 2e-9, 50):g}')
            time.sleep(0.3)
            range_by_vpp(scope)
    for attempt in range(0 if cached and not args.recarrier else 6):
        scope.write(f':TIMebase:SCALe {_snap125(1e-6, 2e-9, 50, up=False):g}')
        time.sleep(0.2)
        if not args.no_autoscale and not getattr(args, '_ranged', False):
            autoscale(scope, cycles=args.cycles, set_timebase=False)   # V/div only
        chans, dt = capture(scope, acq='NORMal', count=args.count)
        if 1 not in chans or 2 not in chans:
            raise RuntimeError("need CH1 (voltage) + CH2 (current) enabled")
        mf = metrics(chans[1] * PROBE_RATIO * args.vmult,
                     chans[2] / COIL_V_PER_A / args.turns, args.load, dt)
        if 1e5 <= mf['Freq_Hz'] <= 1e7:
            carrier, fresh_mf = mf['Freq_Hz'], True
            break
        print(f"  (retry {attempt + 1}: fast window saw {mf['Freq_Hz']:,.0f} Hz -- between bursts)")
    else:
        if not cached or args.recarrier:
            raise RuntimeError("never caught an in-burst carrier in 6 tries -- is the ESU keying?")
    if mf is not None:
        args._carrier = (carrier, mf)

    # 2. LONG window. NORMal, never HRES -- HRES boxcar-averages the carrier to nothing.
    scope.write(f':TIMebase:SCALe {_snap125(args.envwin / HDIV, 2e-9, 50):g}')
    time.sleep(0.3)
    got = float(scope.query(':TIMebase:SCALe?'))
    if got * HDIV < args.envwin * 0.5:
        raise RuntimeError(f"timebase did not take: asked {args.envwin / HDIV:g} s/div, scope is at "
                           f"{got:g} ({got * HDIV * 1e3:.1f} ms window). Set it by hand.")

    # V/div is chosen HERE, on the long window -- the only window that actually contains the
    # envelope peaks. Scaling it on the fast window above picks the range from whatever point
    # of the envelope that 14 us slice happened to land on, and a landing on the flank sets a
    # range the real peaks then rail against. set_timebase=False keeps this long window.
    # NOTE: no autoscale here on purpose. Ranging costs a capture (or four) BEFORE the real
    # one; capturing first and re-ranging only when clip_warn actually fires costs nothing in
    # the common case. The fast-window pass above has already set a sane V/div.

    # ONE DEEP CAPTURE instead of `repeats` shallow ones. Measured on the DSO2C50: 4K memory
    # gives an 80 ms record in 2.3 s, 40K gives 800 ms in 5.4 s -- the sample rate does NOT
    # drop, the record just gets 10x longer. So one 40K frame holds ~96 envelope periods at
    # 120 Hz where three 4K frames held ~10 between them: more data, in a third of the time.
    # The agreement check that `repeats` provided now comes from THIRDS of the one record,
    # which is the same statistic over a longer baseline (267 ms each vs 80 ms).
    prev_depth = None
    try:
        prev_depth = scope.query(':ACQuire:POINts?').strip()
    except Exception:
        pass
    if prev_depth and prev_depth != str(args.envdepth):
        scope.write(f':ACQuire:POINts {int(args.envdepth)}')
        time.sleep(0.6)
    chans, dte = capture(scope, acq='NORMal', count=args.count)
    if clip_warn(scope, chans):
        print("  (re-ranging and re-capturing — a saturated frame reads low)")
        autoscale(scope, cycles=args.cycles, set_timebase=False)
        chans, dte = capture(scope, acq='NORMal', count=args.count)
    # NOT restored here on purpose: the write plus its 0.6 s settle would then be paid again
    # on the very next burst, with the ESU keyed into the load the whole time. fire_loop puts
    # the depth back once when the run ends.

    Vf = chans[1] * PROBE_RATIO * args.vmult
    If = chans[2] / COIL_V_PER_A / args.turns
    m = metrics(Vf, If, args.load, dte)
    n3 = len(Vf) // 3                                     # thirds replace the 3 repeats
    Ps = np.array([float(np.mean(Vf[i*n3:(i+1)*n3] * If[i*n3:(i+1)*n3])) for i in range(3)])
    spread = float(Ps.std() / abs(Ps.mean())) if Ps.mean() else float('nan')
    m['Freq_Hz'] = carrier
    print(f"  carrier {carrier:,.0f} Hz, {len(Vf) * dte * 1e3:.0f} ms record ({len(Vf)} pts) | thirds: "
          + " | ".join(f"{p:.1f}" for p in Ps) + f"  -> {m['P_from_VxI (mean v*i)']:.1f} W (spread {100 * spread:.1f}%)")
    if spread > 0.05:
        print("  WARNING: the thirds of this record disagree >5% -- the window is still shorter "
              "than the modulation period. Raise --envwin or --envdepth; this reading is not trustworthy")
    # Coherent sampling gives a STABLE wrong answer, so `spread` can't see it. The fast capture
    # resolves the carrier, so its Vpeak is the true peak; phase-uniform slow sampling over
    # thousands of samples must still land near it. Much lower = samples stuck at one phase.
    # (A low Vrms/Vpeak ratio does NOT work as the test -- modulated modes read low legitimately.)
    # Only against a Vpeak measured in THIS call: a cached mf came from a different dial
    # setting, so the ratio would be amplitude drift, not missed peaks. Sample rate and carrier
    # are both unchanged between reuses, so coherence cannot newly appear -- the fresh pass settles it.
    hit = m['Vpeak'] / mf['Vpeak'] if fresh_mf and mf['Vpeak'] else float('nan')
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
                # a sweep walks the dial UP, so the range set at the last setting rails at the
                # next one. Re-range and re-capture instead of logging a low number.
                if clip_warn(scope, chans) and not args.no_autoscale:
                    print("  (re-ranging and re-capturing)")
                    autoscale(scope, cycles=args.cycles)
                    chans, dt = capture(scope, acq=args.acq, count=args.count)
                    clip_warn(scope, chans)
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


# ---- machine profiles: refs/<machine>.csv, the same table --compare and --watch already read ----
PROF_HDR = ['model', 'mode', 'setting', 'load_ohm', 'expected_W', 'tol_pct']


def app_dir():
    """Directory to read/write bench data from. Next to the EXE when frozen — a PyInstaller
    onefile unpacks to a temp _MEIPASS that is DELETED on exit, so a profile saved there
    would silently vanish before the tech ever reopened it."""
    return os.path.dirname(sys.executable if getattr(sys, 'frozen', False)
                           else os.path.abspath(__file__))


def refs_dirs():
    """Writable refs dir first, the copy bundled into the exe second (read-only fallback,
    so the shipped Ellman/Surgitron tables are there on a fresh machine)."""
    out = [os.path.join(app_dir(), 'refs')]
    bundled = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'refs')
    if bundled not in out:
        out.append(bundled)
    return out


def band_from(style, a, b):
    """The two ways an OEM states a spec -> the (expected_W, tol_pct) pair the CSV stores.
    'minmax' (Ellman: "17-25 W")  |  'pm' (Valleylab: "300 W +/- 20%").
    Symmetric about the midpoint, so min/max round-trips EXACTLY — see demo()."""
    if style == 'minmax':
        lo, hi = sorted((float(a), float(b)))
        return (lo + hi) / 2, (100.0 * (hi - lo) / (hi + lo) if (hi + lo) else 0.0)
    return float(a), float(b)


def band_to(style, expected_W, tol_pct):
    """Inverse of band_from — render a stored band in whichever style the tech is reading from."""
    if style == 'minmax':
        return expected_W * (1 - tol_pct / 100), expected_W * (1 + tol_pct / 100)
    return expected_W, tol_pct


def profile_slug(name):
    import re
    return re.sub(r'[^a-z0-9]+', '-', str(name).lower()).strip('-') or 'machine'


def list_profiles():
    import glob
    seen = []
    for d in refs_dirs():
        for p in sorted(glob.glob(os.path.join(d, '*.csv'))):
            n = os.path.splitext(os.path.basename(p))[0]
            if n not in seen:
                seen.append(n)
    return seen


def load_profile(name):
    """-> (model, [{mode, setting, load_ohm, expected_W, tol_pct}, ...]) sorted mode then setting."""
    import csv
    for d in refs_dirs():
        p = os.path.join(d, profile_slug(name) + '.csv')
        if os.path.exists(p):
            break
    else:
        return '', []
    rows, model = [], ''
    with open(p, newline='') as f:
        for r in csv.DictReader(f):
            model = model or r.get('model', '')
            rows.append({'mode': r['mode'].strip().lower(), 'setting': float(r['setting']),
                         'load_ohm': float(r['load_ohm']), 'expected_W': float(r['expected_W']),
                         'tol_pct': float(r['tol_pct'])})
    rows.sort(key=lambda r: (r['mode'], r['setting']))
    return model, rows


def save_profile(name, model, rows):
    import csv
    d = refs_dirs()[0]
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, profile_slug(name) + '.csv')
    with open(p, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(PROF_HDR)
        for r in rows:
            w.writerow([model, r['mode'], g(r['setting']), g(r['load_ohm']),
                        g(r['expected_W']), g(r['tol_pct'])])
    return p


def g(x):
    """Trim 3.0 -> 3 so a hand-edited profile CSV stays readable."""
    return int(x) if float(x).is_integer() else x


def ref_bands(ref):
    """Load a reference CSV as {mode: [(setting, lo_W, hi_W)]} — the inverse of the
    expected/tol form, so a measured wattage can be mapped back to candidate dial settings."""
    import csv
    if not os.path.exists(ref):
        name = ref if ref.endswith('.csv') else ref + '.csv'
        for d in refs_dirs():
            if os.path.exists(os.path.join(d, name)):
                ref = os.path.join(d, name); break
        else:
            return {}
    out = {}
    with open(ref, newline='') as f:
        for r in csv.DictReader(f):
            e, t = float(r['expected_W']), float(r['tol_pct'])
            out.setdefault(r['mode'].strip().lower(), []).append(
                (float(r['setting']), e * (1 - t / 100), e * (1 + t / 100)))
    for v in out.values():
        v.sort()
    return out


def identify(bands, mode, watts):
    """Which dial settings could produce this reading? Returns (candidates, note).

    Often the answer is NOT one setting. The OEM bands overlap heavily at the top of every
    mode — on Dento CUT a reading of 55 W sits inside settings 5,6,7,8,9 and 10 at once, so
    claiming a single setting there would be invention, not measurement."""
    b = bands.get(str(mode).strip().lower())
    if not b:
        return None, f"no reference curve for mode '{mode}'"
    hits = [int(s) for s, lo, hi in b if lo <= watts <= hi]
    if not hits:
        # Report the DISTANCE to the nearest band, not just "no match". A reading 0.05 W past
        # a band edge is measurement scatter; one 20 W away is a real fault. Saying
        # "out of spec" for both is how a good unit gets failed.
        near = min(b, key=lambda x: min(abs(watts - x[1]), abs(watts - x[2])))
        gap = min(abs(watts - near[1]), abs(watts - near[2]))
        pct = 100 * gap / watts if watts else float('inf')
        edge = f"setting {int(near[0])} ({near[1]:.1f}-{near[2]:.1f} W)"
        if pct <= 5:
            return [int(near[0])], (f"{watts:.1f} W is {gap:.2f} W ({pct:.1f}%) outside {edge} — "
                                    "within measurement scatter, read it as that setting")
        return [], (f"{watts:.1f} W matches NO setting; nearest is {edge}, {gap:.1f} W ({pct:.0f}%) away. "
                    "Check the load and --turns before calling the unit faulty.")
    if len(hits) == 1:
        return hits, f"unambiguous — setting {hits[0]}"
    return hits, f"AMBIGUOUS — {watts:.1f} W is inside settings {hits}; the OEM bands overlap here"


# DSO2000 V/div gears. Ranging moves in 1-2-5 steps, so "back off two steps" is index math.
GEARS = [m * 10 ** e for e in range(-3, 2) for m in (1, 2, 5)]


def _gear_step(v, gears):
    """The V/div `gears` steps away from `v` (positive = coarser), clamped to the range."""
    i = min(range(len(GEARS)), key=lambda k: abs(GEARS[k] - v))
    return GEARS[max(0, min(len(GEARS) - 1, i + gears))]


def step_vdiv(scope, gears, chans=(1, 2)):
    """Back every channel off by `gears` 1-2-5 steps.

    Run after each burst so the NEXT one starts with headroom. An ESU is walked 0 -> 10, so a
    range that fitted the last setting is guaranteed to clip the next one -- and it clips on
    screen while the tech is holding the pedal, long before any autoscale gets to run. The
    measurement itself re-ranges per burst, so this headroom does not accumulate over a sweep.

    NOT for the detector channel -- see settle() in fire_loop."""
    for ch in chans:
        try:
            s = float(scope.query(f':CHANnel{ch}:SCALe?'))
        except Exception:
            continue
        scope.write(f':CHANnel{ch}:SCALe {_gear_step(s, gears):g}')


def fire_loop(scope, args, stop=None, phase=None, force=None, retarget=None):
    """Generator: arm on the current channel, yield (metrics, railed) for every burst, re-arm.

    The fire-detector shared by --watch (prints + CSV) and the wizard (thread + queue).
    Detection is the CURRENT channel because it is the only genuinely quiet one: measured idle
    it sits at exactly one ADC code (2 mV at 50 mV/div) with zero scatter, while the voltage
    channel carries several volts of pickup. Current only flows when the ESU is actually
    delivering into the load, so it cannot false-trigger on RF pickup.

    Detection runs at ~1 Hz (a :MEASure query costs ~1 s on fw 1.0.8). Ample for a 10 s burst,
    and the latency is a FEATURE: the scope free-runs in AUTO sweep so the frame is always
    current, and capturing ~1 s in skips the turn-on transient.

    `stop` / `force` are callables polled between reads so a GUI can end the loop, or skip a
    stuck release-wait, without killing the thread. `retarget` returns True once when the
    caller has aimed somewhere else while we are ARMED — a ReRead of setting 2 after setting 9
    — and the ranges are re-predicted on the spot, so the scope is already set for the point
    the tech is about to fire, not the one before it. `phase` is called with 'baseline' /
    'armed' / 'capture' / 'release' so a GUI can show the tech what to do with the pedal.
    The caller may mutate `args` (mode/setpoint/load) right after each yield -- that lands
    before the next capture, which is why aiming at the next test point works."""
    import time
    ch = args.fire_ch

    def ph(k):
        if phase:
            phase(k)

    import statistics
    try:
        base_scale = float(scope.query(f':CHANnel{ch}:SCALe?'))   # ONCE, before any :MEASure
    except Exception:
        base_scale = 0.0
    freq_mode = str(args.fire_item).upper().startswith('FREQ')
    args._crest = getattr(args, '_crest', {})    # measured peak/rms per mode, feeds predict_vdiv

    def level():
        """One detector reading, or None if the scope did not answer or answered impossibly.

        In VPP mode the bound comes from the RIG, not from a scope query: at a known load and
        turns count there is a largest Vpp any real ESU mode can produce. Asking the scope for
        a bound is what used to break things -- a timed-out :MEASure is not cancelled, its
        reply lands in the next query, and that is where 4000 V/div came from. drain() now
        drops the orphan, and this is the second line of defence.

        Returns None, not 0.0, so a bad reply cannot read as 'pedal released' either."""
        v = scope_meas(scope, args.fire_item, ch)
        if v is None or freq_mode:
            return v
        cap = vpp_bound(args.max_watts, args.load, args.turns)
        if not 0 <= v <= cap:
            print(f"  (ignoring impossible CH{ch} VPP reply {v:.4g} V — {args.max_watts:g} W into "
                  f"{args.load:g} ohm at {args.turns} turn(s) tops out at {cap:.4g} V)")
            return None
        return v

    thr = 0.0
    if freq_mode:
        # An idle channel measures no frequency AT ALL. Bench-verified on the DSO2C50
        # 2026-09-10, 8 runs: idle read exactly 0.0 every time, keyed read 1.3-2.9 kHz every
        # time. Those keyed numbers are aliased nonsense -- the detector sits at the envelope
        # timebase where a 0.3-4 MHz carrier is ~80x undersampled -- but their EXISTENCE is
        # not nonsense, and existence is the entire question a fire-detector asks.
        #
        # So: no baseline to sit through, no threshold to derive, no idle floor to measure, no
        # V/div to know, and nothing left that a desynced reply can corrupt into a 12 kV trip
        # level. This is strictly less machinery than the VPP path, which stays only as a
        # fallback for a scope where FREQuency turns out not to behave this way.
        print(f"\nDetector: CH{ch} FREQuency — no baseline needed, "
              f"anything above {args.fire_min_hz:g} Hz is a fire")
    else:
        ph('baseline')
        print(f"\nBaselining CH{ch} — DO NOT FIRE for ~4 s...")
        base = [v for v in (level() for _ in range(4)) if v is not None]
        if len(base) < 2:
            raise RuntimeError(f"CH{ch} gave only {len(base)} usable VPP readings out of 4 — check "
                               f"the channel is on and the Pearson is connected")
        # MEDIAN, not max. A real baseline came back min=0.002 max=0.05 — one ADC code and
        # twenty-five of them. Taking the max let the artifact set the trigger level.
        quiet = statistics.median(base)
        if quiet <= 0:
            raise RuntimeError(f"CH{ch} reads 0 V idle — the detector has nothing to work from.")
        thr = args.fire_threshold or quiet * args.fire_gain
        amps = thr / COIL_V_PER_A / args.turns
        print(f"  idle CH{ch} Vpp: min={min(base):.4g} median={quiet:.4g} max={max(base):.4g} V")
        print(f"  ARMED — fires above {thr:.4g} V = {args.fire_gain:g}x idle "
              f"(~{amps:.3g} A, ~{amps ** 2 * args.load:.2f} W into {args.load:g} ohm)")
    fired, released = fire_test(args.fire_item, thr, args.fire_min_hz)
    ph('armed')

    def settle():
        """Between bursts: detector channel back on its sensitive BASELINE range, headroom on
        the others.

        A widened detector is a deaf detector. CH2 at 50 mV/div trips at ~3 ADC codes = 60 mA
        = ~1.8 W into 500 ohm; back it off the same 2 gears as CH1 and the same 3 codes are
        ~29 W, so every low setting walks straight past it. So the detector does NOT get
        headroom -- it will visibly rail on screen when the pedal goes down at a high setting,
        and that is fine: it is a trigger, not a measurement. The real capture re-ranges both
        channels before it reads anything. The threshold is a fraction of screen so it would
        survive the detector being left wide, but sensitivity would not: at 4x the range, the
        same 1% of screen is 4x the current."""
        crest = args._crest.get(str(args.mode)) or args.crest_hint
        want = predict_vdiv(getattr(args, 'expect_w', 0.0), args.load, args.turns, args.vmult, crest)
        args._ranged = bool(want)
        if want:
            # Both channels, from the profile's expected watts, with the pedal UP.
            scope.write(f':CHANnel1:SCALe {want[0]:g}')
            scope.write(f':CHANnel2:SCALe {want[1]:g}')
            print(f"  pre-ranged for {getattr(args, 'expect_w', 0.0):.4g} W "
                  f"(crest {crest:.2g}): CH1 {want[0]:g} V/div, CH2 {want[1]:g} V/div")
            return
        if base_scale:
            scope.write(f':CHANnel{ch}:SCALe {base_scale:g}')   # write, never query: no reply to desync
        others = [c for c in (1, 2) if c != ch]
        if args.headroom and others:
            step_vdiv(scope, args.headroom, chans=others)
            print(f"  detector CH{ch} back to {base_scale:g} V/div; "
                  f"CH{others[0]} backed off {args.headroom} step(s) so the next setting cannot clip")

    def rearm():
        ph('release')
        print("  release the pedal to re-arm...")
        t0, said, quiet_runs = time.time(), 0, 0
        while not (stop and stop()):
            if force and force():
                print("  re-armed by hand"); break
            v = level()
            if released(v):
                quiet_runs += 1
                if quiet_runs >= 2:      # two in a row: one dropped reply must not re-arm us
                    break
                continue
            quiet_runs = 0
            if v is None:
                continue
            if time.time() - t0 > 6 * (said + 1):
                said += 1
                print(f"  still keyed: CH{ch} {args.fire_item} reads {v:.6g}")
        ph('armed')
        print("  ARMED\n")

    if getattr(args, 'expect_w', 0.0):
        settle()                                 # burst 1 gets pre-ranged too, not just 2+
    n = 0
    while not (stop and stop()):
        if retarget and retarget():
            settle()                             # aimed elsewhere while armed: re-range now
        v = level()
        if not fired(v):
            continue
        n += 1
        ph('capture')
        t_key = time.time()
        hold = "~10 s on the first burst, ~6 s after" if args.envelope else "~5 s"
        print(f"--- BURST {n} detected --- capturing (keep it keyed {hold})")
        if args.envelope:
            # Long-window mean(v*i) -- the only honest average for a MODULATED mode.
            # envelope_power re-ranges and re-captures itself, and caches the carrier, so
            # only the first burst pays the ~7 s hunt. A burst released too early raises
            # instead of logging a number read out of a gap: drop it and re-arm.
            try:
                m, _, _ = envelope_power(scope, args)
            except RuntimeError as e:
                print(f"  burst {n} DISCARDED: {e}")
                n -= 1; settle(); rearm(); continue
            railed = False
        else:
            chans, dt = capture(scope, acq='NORMal', count=args.count)
            railed = clip_warn(scope, chans)
            V = chans[1] * PROBE_RATIO * args.vmult
            I = chans[2] / COIL_V_PER_A / args.turns
            m = metrics(V, I, args.load, dt)
        # ponytail: the scope's own VRMS/FREQ/MATH reads used to live here and are GONE.
        # Each costs ~1 s of a 10 s burst and all three were proven worthless on live RF
        # 2026-09-09: VRMS returns a frequency (one burst gave VRMS == FREQ exactly), FREQ
        # is an alias at any envelope timebase, and MATH VAVG read 0.000e+00 through four
        # real bursts. --math still forces the MATH read for anyone re-testing that claim.
        print(f"  P: Vrms2/R={m['P_from_V (Vrms^2/R)']:.1f}  Irms2*R={m['P_from_I (Irms^2*R)']:.1f}"
              f"  v*i={m['P_from_VxI (mean v*i)']:.1f} W   Vrms={m['Vrms']:.1f} Irms={m['Irms']:.3f}"
              f" f={m['Freq_Hz']:,.0f}Hz ph={m['Phase_deg']:.0f}deg")
        if args.math:
            print(f"  scope MATH VAVG={scope_meas(scope, 'VAVG', 'MATH')}  (diagnostic; proven dead on fw 1.0.8)")
        warn_modulated(m)
        cf = m.get('CrestFactor')
        if cf and 1.0 < cf < 12:                # remember what THIS mode really looks like
            args._crest[str(args.mode)] = cf
        # The number that matters for the heatsink: how long the ESU was actually keyed.
        print(f"  burst window {time.time() - t_key:.1f} s")
        yield m, railed
        if railed:
            print("  !! that burst was SATURATED — re-ranging now, fire again for a valid number")
            autoscale(scope, cycles=args.cycles)
        settle()
        rearm()


def watch(args):
    """--watch: fire-detect, capture on its own, log each burst to CSV, re-arm.
    Combine with --envelope for a long-window average on modulated modes."""
    import csv, os, time
    scope = connect(args.resource)
    fh = None
    restore = {}
    try:
        for k in (':TRIGger:SWEep?', ':ACQuire:POINts?'):   # envelope mode leaves POINts deep
            try: restore[k] = scope.query(k).strip()
            except Exception: pass
        drain(scope)                             # drop any stale reply before :MEASure polling
        scope.write(':TRIGger:SWEep AUTO')       # free-run, so the frame is always fresh
        if args.math:                            # opt-in diagnostic only
            scope.write(':MATH:OPERator MULTiply')
            scope.write(':MATH:DISPlay ON')

        bands = ref_bands(args.ref)
        if bands:
            print(f"  reference: {args.ref}  (modes: {', '.join(sorted(bands))})")
        csvpath = (args.session_name or 'esu_watch') + '_bursts.csv'
        HDR = ['t', 'mode', 'setting', 'load_ohm', 'P_Vrms2R', 'P_Irms2R', 'P_meanVI',
               'Vrms', 'Irms', 'Freq_Hz', 'Phase_deg', 'clipped', 'settings']
        # An older log has the three dead scope columns. Appending rows of a different width
        # would silently misalign it, so move it aside rather than corrupt real bench data.
        if os.path.exists(csvpath):
            with open(csvpath, newline='') as _f:
                first = next(csv.reader(_f), [])
            if first and first != HDR:
                os.replace(csvpath, csvpath + '.old')
                print(f"  NOTE: {csvpath} had the old column set — kept as {csvpath}.old")
        new_file = not os.path.exists(csvpath)
        fh = open(csvpath, 'a', newline=''); wr = csv.writer(fh)
        if new_file: wr.writerow(HDR); fh.flush()
        print(f"  logging to {csvpath}   (Ctrl-C to stop)")

        args.expect_w = expected_w(bands, args.mode, args.setpoint)
        for m, railed in fire_loop(scope, args):
            hits = None
            if bands:
                # grade on mean(v*i): it assumes neither the typed load nor a resistive one
                hits, note = identify(bands, args.mode, m['P_from_VxI (mean v*i)'])
                print(f"  setting: {note}")
            wr.writerow([time.strftime('%H:%M:%S'), args.mode, args.setpoint, args.load,
                         m['P_from_V (Vrms^2/R)'], m['P_from_I (Irms^2*R)'],
                         m['P_from_VxI (mean v*i)'], m['Vrms'], m['Irms'], m['Freq_Hz'],
                         m['Phase_deg'], bool(railed),
                         ' '.join(map(str, hits)) if hits else '']); fh.flush()
            # --mode/--setpoint can be edited between bursts; re-look the expected watts so the
            # next burst is pre-ranged for the point actually being tested.
            args.expect_w = expected_w(bands, args.mode, args.setpoint)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        try: fh.close()
        except Exception: pass
        for k, v in restore.items():
            try: scope.write(k[:-1] + ' ' + v)
            except Exception: pass
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


class _QueueWriter:
    """Line-buffered file object that forwards print() to the GUI log.
    ponytail: exists so every existing diagnostic print (clip warnings, envelope spread,
    coherent-sampling warning) shows up in the wizard instead of a console the exe has none of."""
    def __init__(self, q): self.q, self.buf = q, ''
    def write(self, s):
        self.buf += s
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            self.q.put(('log', line))
    def flush(self): pass


def wizard():
    """The tech workflow, one window: build or open a machine profile, walk mode x setting
    with fire-detect, grade every point against the profile, re-read what looks wrong.

    Stdlib tkinter/ttk only — no new dependency, ships inside the existing exe."""
    import tkinter as tk
    from tkinter import ttk, messagebox
    import contextlib, csv, queue, threading, types

    S = types.SimpleNamespace(rows=[], meas={}, cursor=None, stop=True, thread=None,
                              q=queue.Queue(), style=None, force=False, reaim=False)

    root = tk.Tk(); root.title("ESU Calibration Wizard")
    root.geometry("1000x820")
    S.style = tk.StringVar(value='minmax')
    nb = ttk.Notebook(root); nb.pack(fill='both', expand=True)

    # ================= tab 1: machine profile =================
    pf = ttk.Frame(nb); nb.add(pf, text=' 1. Machine profile ')

    bar = ttk.Frame(pf); bar.pack(fill='x', padx=8, pady=(8, 2))
    ttk.Label(bar, text="Machine").pack(side='left')
    machine = ttk.Combobox(bar, values=list_profiles(), width=30); machine.pack(side='left', padx=6)
    ttk.Label(bar, text="Model / label").pack(side='left', padx=(12, 4))
    model = ttk.Entry(bar, width=30); model.pack(side='left')

    sty = ttk.Frame(pf); sty.pack(fill='x', padx=8, pady=2)
    ttk.Label(sty, text="Spec is written as:").pack(side='left')

    ptv = ttk.Treeview(pf, columns=('mode', 'setting', 'load', 'a', 'b'),
                       show='headings', height=13, selectmode='extended')
    for c, w in (('mode', 130), ('setting', 80), ('load', 90), ('a', 130), ('b', 130)):
        ptv.column(c, width=w, anchor='center')
    ptv.heading('mode', text='Mode'); ptv.heading('setting', text='Setting')
    ptv.heading('load', text='Load Ω')
    ptv.pack(fill='both', expand=True, padx=8, pady=6)

    def prender():
        sel = set(ptv.selection())
        ptv.delete(*ptv.get_children())
        mm = S.style.get() == 'minmax'
        ptv.heading('a', text='Min W' if mm else 'Nominal W')
        ptv.heading('b', text='Max W' if mm else '± %')
        for i, r in enumerate(S.rows):
            a, b = band_to(S.style.get(), r['expected_W'], r['tol_pct'])
            ptv.insert('', 'end', iid=str(i), values=(r['mode'], f"{g(r['setting'])}",
                       f"{g(r['load_ohm'])}", f"{a:.4g}", f"{b:.4g}"))
        ptv.selection_set([i for i in sel if i in ptv.get_children()])
        rsync()

    for txt, val in (("min – max   (Ellman)", 'minmax'), ("nominal ± %   (Valleylab)", 'pm')):
        ttk.Radiobutton(sty, text=txt, value=val, variable=S.style,
                        command=prender).pack(side='left', padx=8)

    addf = ttk.LabelFrame(pf, text="Add a mode")
    addf.pack(fill='x', padx=8, pady=4)
    ttk.Label(addf, text="Mode").grid(row=0, column=0, padx=4, pady=6)
    e_mode = ttk.Entry(addf, width=14); e_mode.grid(row=0, column=1)
    ttk.Label(addf, text="settings").grid(row=0, column=2, padx=(12, 2))
    e_lo = ttk.Entry(addf, width=5); e_lo.insert(0, '0'); e_lo.grid(row=0, column=3)
    ttk.Label(addf, text="to").grid(row=0, column=4, padx=2)
    e_hi = ttk.Entry(addf, width=5); e_hi.insert(0, '10'); e_hi.grid(row=0, column=5)
    ttk.Label(addf, text="load Ω").grid(row=0, column=6, padx=(12, 2))
    e_ld = ttk.Entry(addf, width=7); e_ld.insert(0, '500'); e_ld.grid(row=0, column=7)

    def add_mode():
        try:
            lo, hi, ld = int(e_lo.get()), int(e_hi.get()), float(e_ld.get())
        except ValueError:
            return messagebox.showerror("Add mode", "settings and load must be numbers")
        name = e_mode.get().strip().lower()
        if not name:
            return messagebox.showerror("Add mode", "name the mode (cut, cutcoag, coag, fulg …)")
        have = {(r['mode'], r['setting']) for r in S.rows}
        for s in range(lo, hi + 1):
            if (name, float(s)) not in have:
                S.rows.append({'mode': name, 'setting': float(s), 'load_ohm': ld,
                               'expected_W': 0.0, 'tol_pct': 0.0})
        S.rows.sort(key=lambda r: (r['mode'], r['setting']))
        prender()

    ttk.Button(addf, text="Add", command=add_mode).grid(row=0, column=8, padx=10)

    setf = ttk.LabelFrame(pf, text="Set the spec for the selected row(s)")
    setf.pack(fill='x', padx=8, pady=(0, 8))
    l_a = ttk.Label(setf, text="Min W"); l_a.grid(row=0, column=0, padx=4, pady=6)
    e_a = ttk.Entry(setf, width=9); e_a.grid(row=0, column=1)
    l_b = ttk.Label(setf, text="Max W"); l_b.grid(row=0, column=2, padx=(12, 2))
    e_b = ttk.Entry(setf, width=9); e_b.grid(row=0, column=3)
    ttk.Label(setf, text="load Ω (blank = keep)").grid(row=0, column=4, padx=(12, 2))
    e_l2 = ttk.Entry(setf, width=7); e_l2.grid(row=0, column=5)

    def apply_spec():
        sel = ptv.selection()
        if not sel:
            return messagebox.showinfo("Set spec", "select one or more rows first")
        try:
            exp, tol = band_from(S.style.get(), float(e_a.get()), float(e_b.get()))
        except ValueError:
            return messagebox.showerror("Set spec", "both values must be numbers")
        ld = e_l2.get().strip()
        for iid in sel:
            r = S.rows[int(iid)]
            r['expected_W'], r['tol_pct'] = exp, tol
            if ld:
                r['load_ohm'] = float(ld)
        prender()

    ttk.Button(setf, text="Apply", command=apply_spec).grid(row=0, column=6, padx=10)

    def drop_rows():
        for iid in sorted((int(i) for i in ptv.selection()), reverse=True):
            del S.rows[iid]
        S.meas.clear(); S.cursor = None
        prender()

    ttk.Button(setf, text="Delete selected", command=drop_rows).grid(row=0, column=7, padx=4)

    def do_open():
        name = machine.get().strip()
        if not name:
            return messagebox.showinfo("Open", "pick or type a machine name")
        mdl, rows = load_profile(name)
        if not rows:
            return messagebox.showerror("Open", f"no saved profile for '{name}'")
        S.rows, S.meas, S.cursor = rows, {}, None
        model.delete(0, 'end'); model.insert(0, mdl)
        prender()

    def do_save():
        name = machine.get().strip()
        if not name:
            return messagebox.showinfo("Save", "name the machine first")
        if not S.rows:
            return messagebox.showinfo("Save", "nothing to save")
        p = save_profile(name, model.get().strip() or name, S.rows)
        machine.configure(values=list_profiles())
        messagebox.showinfo("Saved", p)

    ttk.Button(bar, text="Open", command=do_open).pack(side='left', padx=(12, 4))
    ttk.Button(bar, text="Save", command=do_save).pack(side='left')

    # ================= tab 2: run =================
    rf = ttk.Frame(nb); nb.add(rf, text=' 2. Run ')
    f1 = ttk.Frame(rf); f1.pack(fill='x', padx=8, pady=8)
    fields = {}
    for i, (lbl, key, dflt, w) in enumerate((("Unit S/N", 'sn', '', 16), ("Coil turns", 'turns', '1', 5),
                                             ("V-tap mult", 'vmult', '1.0', 5), ("Fire CH", 'fire_ch', '2', 4),
                                             ("Headroom steps", 'headroom', '2', 4))):
        ttk.Label(f1, text=lbl).grid(row=0, column=2 * i, padx=(0 if i == 0 else 12, 4))
        e = ttk.Entry(f1, width=w); e.insert(0, dflt); e.grid(row=0, column=2 * i + 1)
        fields[key] = e
    env = tk.BooleanVar(value=True)
    ttk.Checkbutton(f1, text="Envelope (modulated modes)", variable=env).grid(row=0, column=10, padx=14)

    # Colour, not wording, is what a tech reads from across the bench — the ESU is usually
    # on a different table from the PC. Green = key it, red = let go, blue = do not move.
    PHASE = {'idle':     ('#4a4a4a', "IDLE"),
             'baseline': ('#8a6d00', "BASELINING — DO NOT FIRE"),
             'armed':    ('#12802f', "▶  PRESS THE PEDAL"),
             'capture':  ('#0b4f9e', "■  HOLD IT — CAPTURING"),
             'release':  ('#a51c1c', "✋  RELEASE THE PEDAL"),
             'done':     ('#12802f', "ALL POINTS MEASURED")}
    ban = tk.Frame(rf, bg=PHASE['idle'][0])
    ban.pack(fill='x', padx=8, pady=6)
    b_act = tk.Label(ban, text=PHASE['idle'][1], font=("", 26, "bold"), bg=PHASE['idle'][0], fg='white')
    b_act.pack(pady=(12, 0))
    b_pt = tk.Label(ban, text="— open a profile on tab 1 —", font=("", 40, "bold"),
                    bg=PHASE['idle'][0], fg='white')
    b_pt.pack()
    b_exp = tk.Label(ban, text="", font=("", 15), bg=PHASE['idle'][0], fg='#e8e8e8')
    b_exp.pack(pady=(0, 12))

    def setphase(k):
        bg, txt = PHASE[k]
        for w in (ban, b_act, b_pt, b_exp):
            w.configure(bg=bg)
        b_act.configure(text=txt)

    rtv = ttk.Treeview(rf, columns=('mode', 'setting', 'exp', 'meas', 'verdict'),
                       show='headings', height=12, selectmode='extended')
    for c, t, w in (('mode', 'Mode', 120), ('setting', 'Setting', 80), ('exp', 'Expected', 170),
                    ('meas', 'Measured W', 110), ('verdict', 'Verdict', 200)):
        rtv.heading(c, text=t); rtv.column(c, width=w, anchor='center')
    rtv.tag_configure('pass', background='#d8f5d8')
    rtv.tag_configure('fail', background='#f8d8d8')
    rtv.tag_configure('aim', background='#fff3c4')
    rtv.pack(fill='both', expand=True, padx=8)

    def target():
        """Row the next burst lands on: an explicit ReRead pick, else the first unmeasured."""
        if S.cursor is not None and S.cursor < len(S.rows):
            return S.cursor
        return next((i for i in range(len(S.rows)) if i not in S.meas), None)

    def rsync():
        rtv.delete(*rtv.get_children())
        aim = target()
        for i, r in enumerate(S.rows):
            lo, hi = band_to('minmax', r['expected_W'], r['tol_pct'])
            m = S.meas.get(i)
            if m is None:
                w, vd, tag = '', '', ('aim' if i == aim else '')
            else:
                w = m['P_from_VxI (mean v*i)']
                ok, _, _, dev = verdict(w, r['expected_W'], r['tol_pct'])
                spec = bool(r['expected_W'])
                vd = f"{'PASS' if ok else 'FAIL'}  ({dev:+.0f}%)" if spec else 'no spec'
                w, tag = f"{w:.1f}", (('pass' if ok else 'fail') if spec else '')
                if i == aim:
                    tag = 'aim'
            rtv.insert('', 'end', iid=str(i), values=(r['mode'], g(r['setting']),
                       f"{lo:.4g} – {hi:.4g} W @ {g(r['load_ohm'])}Ω", w, vd), tags=(tag,))
        # Text only — the colour is driven by the pedal phase, so the two never fight.
        if not S.rows:
            b_pt.configure(text="— open a profile on tab 1 —"); b_exp.configure(text="")
        elif aim is None:
            b_pt.configure(text="ALL POINTS MEASURED")
            b_exp.configure(text="save the results, or select a row and hit ReRead")
        else:
            r = S.rows[aim]
            lo, hi = band_to('minmax', r['expected_W'], r['tol_pct'])
            b_pt.configure(text=f"{r['mode'].upper()}   ·   LEVEL {g(r['setting'])}")
            b_exp.configure(text=f"expect {lo:.4g} – {hi:.4g} W  into {g(r['load_ohm'])} Ω")
            rtv.see(str(aim))

    def reread():
        sel = rtv.selection()
        if not sel:
            return messagebox.showinfo("ReRead", "select the row to read again")
        for iid in sel:
            S.meas.pop(int(iid), None)
        S.cursor = int(sel[0])
        S.reaim = True          # re-range the scope for it now, not at the next burst
        rsync()

    def rerun_mode():
        sel = rtv.selection()
        if not sel:
            return messagebox.showinfo("Re-run mode", "select any row of the mode to re-run")
        mode = S.rows[int(sel[0])]['mode']
        idxs = [i for i, r in enumerate(S.rows) if r['mode'] == mode]
        for i in idxs:
            S.meas.pop(i, None)
        S.cursor = idxs[0]
        S.reaim = True
        rsync()

    logbox = tk.Text(rf, height=7, wrap='none', bg='#111', fg='#cfc', font=('Consolas', 11))
    logbox.pack(fill='both', expand=False, padx=8, pady=6)

    def log(line):
        logbox.insert('end', line + '\n'); logbox.see('end')

    def worker(args):
        # ponytail: redirect_stdout is process-global, which is safe only because this is the
        # ONE worker thread and the Tk thread never prints. Two workers would interleave.
        with contextlib.redirect_stdout(_QueueWriter(S.q)):
            scope = None
            try:
                scope = connect(args.resource)
                try: depth0 = scope.query(':ACQuire:POINts?').strip()
                except Exception: depth0 = None
                drain(scope)                     # drop any stale reply before :MEASure polling
                scope.write(':TRIGger:SWEep AUTO')

                def aim():
                    i = target()
                    if i is not None:
                        r = S.rows[i]
                        args.mode, args.setpoint, args.load = r['mode'], g(r['setting']), r['load_ohm']
                        args.expect_w = r['expected_W']      # pre-range while the pedal is up
                    S.q.put(('sync',))

                def took_force():
                    if S.force:
                        S.force = False
                        return True
                    return False

                def took_reaim():
                    if S.reaim:
                        S.reaim = False
                        aim()                    # new expected watts -> new predicted ranges
                        return True
                    return False

                aim()
                for m, railed in fire_loop(scope, args, stop=lambda: S.stop, force=took_force,
                                           retarget=took_reaim,
                                           phase=lambda k: S.q.put(('phase', k))):
                    i = target()
                    if i is None:
                        print("  no pending point — pick a row and hit ReRead")
                    else:
                        # meas/cursor are also touched by ReRead on the Tk thread. Every op
                        # here is a single atomic dict/attr assignment, so the only possible
                        # loss is a ReRead pressed in the same instant a burst lands -- press
                        # it again. ponytail: a lock buys nothing a second click does not.
                        S.meas[i] = m
                        S.cursor = None
                    aim()
            except Exception as e:
                print(f"STOPPED: {e}")
            finally:
                S.stop = True
                try: scope.write(f':ACQuire:POINts {depth0}')   # envelope mode left it deep
                except Exception: pass
                try: scope.close()
                except Exception: pass
                S.q.put(('sync',))
                S.q.put(('done',))

    def start():
        if S.thread and S.thread.is_alive():
            return
        if not S.rows:
            return messagebox.showerror("Start", "open or build a machine profile on tab 1 first")
        try:
            args = make_parser().parse_args([])      # every default, from the one place
            args.load = S.rows[0]['load_ohm']
            args.turns = int(fields['turns'].get() or 1)
            args.vmult = float(fields['vmult'].get() or 1.0)
            args.fire_ch = int(fields['fire_ch'].get() or 2)
            args.headroom = int(fields['headroom'].get() or 0)
            args.envelope = bool(env.get())
            args.acq = 'NORMal'
        except ValueError:
            return messagebox.showerror("Start", "turns / mult / channel must be numbers")
        S.stop = False
        b_start.state(['disabled']); b_stop.state(['!disabled'])
        S.thread = threading.Thread(target=worker, args=(args,), daemon=True)
        S.thread.start()

    def stop():
        S.stop = True
        setphase('idle')
        log("stopping after the current burst…")

    def force_rearm():
        S.force = True
        log("forcing re-arm…")

    def save_results():
        done = [i for i in range(len(S.rows)) if i in S.meas]
        if not done:
            return messagebox.showinfo("Save", "nothing measured yet")
        sn = (fields['sn'].get().strip() or 'NA').replace(' ', '_')
        p = os.path.join(app_dir(), f"{profile_slug(machine.get() or 'machine')}_{sn}_results.csv")
        with open(p, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sn', 'model', 'mode', 'setting', 'load_ohm', 'expected_W', 'tol_pct',
                        'measured_W', 'pass', 'Vrms', 'Irms', 'Freq_Hz', 'Phase_deg'])
            for i in done:
                r, m = S.rows[i], S.meas[i]
                got = m['P_from_VxI (mean v*i)']
                ok = verdict(got, r['expected_W'], r['tol_pct'])[0] if r['expected_W'] else ''
                w.writerow([sn, model.get(), r['mode'], g(r['setting']), g(r['load_ohm']),
                            g(r['expected_W']), g(r['tol_pct']), round(got, 2), ok,
                            round(m['Vrms'], 1), round(m['Irms'], 4), round(m['Freq_Hz']),
                            round(m['Phase_deg'], 1)])
        messagebox.showinfo("Saved", p)

    btns = ttk.Frame(rf); btns.pack(fill='x', padx=8, pady=(0, 8))
    b_start = ttk.Button(btns, text="Start (arm fire-detect)", command=start); b_start.pack(side='left')
    b_stop = ttk.Button(btns, text="Stop", command=stop); b_stop.pack(side='left', padx=6)
    b_stop.state(['disabled'])
    ttk.Button(btns, text="ReRead selected", command=reread).pack(side='left', padx=(20, 6))
    ttk.Button(btns, text="Re-run whole mode", command=rerun_mode).pack(side='left')
    ttk.Button(btns, text="Force re-arm", command=force_rearm).pack(side='left', padx=6)
    ttk.Button(btns, text="Save results CSV", command=save_results).pack(side='right')

    def pump():
        try:
            while True:
                msg = S.q.get_nowait()
                if msg[0] == 'log': log(msg[1])
                elif msg[0] == 'sync': rsync()
                elif msg[0] == 'phase': setphase(msg[1])
                elif msg[0] == 'done':
                    b_start.state(['!disabled']); b_stop.state(['disabled'])
                    setphase('idle'); log("— stopped —")
        except queue.Empty:
            pass
        root.after(120, pump)

    prender()
    root.after(120, pump)
    root.mainloop()
    S.stop = True


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
    # clip_state: a clean sine must NOT read as railed; a flat-topped one must.
    # 1 V/div, offset 0 -> the ADC rails at ADC_RAIL/CODES_PER_DIV = 5.08 V.
    clean = 4.0 * np.sin(2 * np.pi * 4000 * t)                    # peak 100 codes = the screen edge
    top, railed = clip_state(clean, 1.0, 0.0)
    assert railed == 0.0 and abs(top - 100) < 1, (top, railed)
    over = 5.0 * np.sin(2 * np.pi * 4000 * t)                     # past the screen, still under the rail
    top, railed = clip_state(over, 1.0, 0.0)
    assert railed == 0.0 and top > 4 * CODES_PER_DIV, (top, railed)   # valid data, only LOOKS clipped
    sat = np.clip(8.0 * np.sin(2 * np.pi * 4000 * t), -5.08, 5.08)    # genuinely flat-topped
    top, railed = clip_state(sat, 1.0, 0.0)
    assert railed > 0.3, railed
    # and the failure this exists to catch: a saturated capture under-reports power
    assert np.sqrt(np.mean(sat**2)) < 0.95 * np.sqrt(np.mean((8.0 * np.sin(2 * np.pi * 4000 * t))**2))
    # identify(): the OEM bands overlap, so a reading often maps to SEVERAL settings.
    B = {'fulg': [(0,0,4),(1,0,4),(2,6,8),(3,7,11),(4,17,25),(5,21,31),(6,25,37)]}
    assert identify(B, 'fulg', 18.34)[0] == [4]              # unambiguous
    assert identify(B, 'fulg', 28.62)[0] == [5, 6]           # overlap -- must NOT claim one
    assert identify(B, 'fulg', 10.84)[0] == [3]
    # 11.05 W is 0.05 W past setting 3's edge -- scatter, NOT a fault. Must not fail a good unit.
    assert identify(B, 'fulg', 11.05)[0] == [3] and 'scatter' in identify(B, 'fulg', 11.05)[1]
    assert identify(B, 'fulg', 99.0)[0] == [] and 'NO setting' in identify(B, 'fulg', 99.0)[1]
    assert identify(B, 'fulg', 14.0)[0] == []                # a real 3 W gap stays unmatched
    assert identify(B, 'cut', 10)[0] is None                 # mode absent from this table
    # a band round-trips exactly through the expected_W/tol_pct form the ref CSV stores
    b_lo, b_hi = 17, 25                       # named so they cannot shadow the demo's t/V/I
    b_exp = (b_lo + b_hi) / 2
    b_tol = 100.0 * (b_hi - b_lo) / (b_hi + b_lo)
    assert abs(b_exp * (1 - b_tol/100) - b_lo) < 1e-9 and abs(b_exp * (1 + b_tol/100) - b_hi) < 1e-9
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
    # thirds-of-one-record must reproduce what 3 separate captures reported, and a record
    # SHORTER than the modulation period must still be flagged. This is the check that the
    # envelope speed-up did not quietly trade accuracy for wall-clock.
    # 3.9e6 / 50e3 = EXACTLY 78, which makes every sample land at the same carrier phase and
    # the product average to zero -- the coherent-sampling trap this code warns about. Use a
    # carrier that is not a whole multiple of the sample rate, as a real machine never is.
    fenv, Rr, fcar = 120.0, 500.0, 3913700.0
    # 10K memory (a 200 ms record) is the DEFAULT: every extra sample is more time the ESU is
    # keyed and the heatsink is drifting. 200 ms is not an arbitrary choice -- it is exactly 12
    # mains periods at 60 Hz AND 10 at 50 Hz, so a mains-locked envelope fits a WHOLE number of
    # periods either way and there is no partial-period error. 120 ms is 7.2 periods at 60 Hz
    # and reads ~1.5% high on half-wave coag; that is the failure this pins down.
    for shape, ef in (('full-wave blend', 120.0), ('half-wave coag', 60.0)):
        truth = None
        for dur in (4.0, 800e-3, 400e-3, 200e-3):          # 4 s = the reference answer
            tl = np.arange(0, dur, 1 / 50e3)
            env = (np.abs(np.sin(2 * np.pi * 0.5 * ef * tl * 2)) if ef == 120.0
                   else np.clip(np.sin(2 * np.pi * ef * tl), 0, None))
            Vl = 300 * np.sqrt(2) * env * np.sin(2 * np.pi * fcar * tl)
            Il = Vl / Rr
            P = float(np.mean(Vl * Il))
            if truth is None:
                truth = P
                continue
            assert abs(P / truth - 1) < 0.02, (shape, dur, P, truth)
            n3 = len(Vl) // 3
            thirds = np.array([np.mean(Vl[i*n3:(i+1)*n3] * Il[i*n3:(i+1)*n3]) for i in range(3)])
            assert thirds.std() / thirds.mean() < 0.02, (shape, dur, thirds)
        # and the length that does NOT fit whole periods must be visibly worse
        tb = np.arange(0, 120e-3, 1 / 50e3)
        envb = (np.abs(np.sin(2 * np.pi * ef * tb)) if ef == 120.0
                else np.clip(np.sin(2 * np.pi * ef * tb), 0, None))
        Vb = 300 * np.sqrt(2) * envb * np.sin(2 * np.pi * fcar * tb)
        n3 = len(Vb) // 3
        tb3 = np.array([np.mean(Vb[i*n3:(i+1)*n3] ** 2 / Rr) for i in range(3)])
        if ef == 60.0:
            assert tb3.std() / tb3.mean() > 0.05, tb3     # 120 ms half-wave: thirds disagree
    ts = np.arange(0, 8e-3, 1 / 50e3)                      # 8 ms: SHORTER than one envelope period
    Vs = 300 * np.sqrt(2) * np.abs(np.sin(2 * np.pi * fenv * ts)) * np.sin(2 * np.pi * fcar * ts)
    Is = Vs / Rr
    m3 = len(Vs) // 3
    sh = np.array([np.mean(Vs[i*m3:(i+1)*m3] * Is[i*m3:(i+1)*m3]) for i in range(3)])
    assert sh.std() / abs(sh.mean()) > 0.05, sh            # too-short window MUST trip the warning
    # and the failure the old peak-detect math produced on CW: exactly HALF
    Vm = 300 * np.sqrt(2) * np.sin(2 * np.pi * fc * tc); Im = Vm / R
    dec = int(round((1 / 50e3) / dtc))
    half = np.mean(np.abs(Vm[::dec]) * np.abs(Im[::dec])) / 2
    assert abs(half / np.mean(Vm * Im) - 0.5) < 0.02, half   # the 50% underread, reproduced
    # ---- pre-ranging from expected watts: the ranging that never happens is the fastest ----
    # Ellman cut ~50 W into 500 ohm, 3 turns through the Pearson, 1000:1 probe reported by the
    # scope so vmult=1. Peak should land near 3 of 8 divisions on both channels.
    v1, v2 = predict_vdiv(50, 500, 3, 1.0, crest=1.41)         # cut is CW: crest = sqrt(2)
    assert 2.0 <= 1.41 * math.sqrt(50 * 500) / v1 <= 4.0, (v1, 1.41 * math.sqrt(50 * 500) / v1)
    assert 2.0 <= 1.41 * math.sqrt(50 / 500) * COIL_V_PER_A * 3 / v2 <= 4.0, v2
    assert predict_vdiv(0, 500, 1, 1.0) is None and predict_vdiv(50, 0, 1, 1.0) is None
    # more watts must never mean a tighter range, and the ranges must track turns and vmult
    assert predict_vdiv(200, 500, 3, 1.0)[0] >= predict_vdiv(50, 500, 3, 1.0)[0]
    assert predict_vdiv(50, 500, 6, 1.0)[1] >= predict_vdiv(50, 500, 3, 1.0)[1]
    assert predict_vdiv(50, 500, 3, 3.0)[0] <= predict_vdiv(50, 500, 3, 1.0)[0]  # probing 1/3 of the load
    # a railed frame must stay possible to detect: the prediction leaves room to the ADC rail
    assert vpp_ceiling(v1) > 2 * 1.41 * math.sqrt(50 * 500), (vpp_ceiling(v1),)
    # expected_w pulls the band midpoint back out of a ref table, and says 0 when it cannot
    B = {'fulg': [(4.0, 17.0, 25.0), (5.0, 21.0, 31.0)]}
    assert expected_w(B, 'fulg', 4) == 21.0 and expected_w(B, 'FULG', '5') == 26.0
    assert expected_w(B, 'fulg', 9) == 0.0 and expected_w(B, 'cut', 4) == 0.0
    assert expected_w(B, 'fulg', '?') == 0.0 and expected_w({}, 'fulg', 4) == 0.0
    # ---- a bare number in the reply queue is a stray measurement, never an identity ----
    # "1.316e+03" is a FREQuency reply that answered *IDN?. The first version of this test
    # asked `any(c.isalpha())` and passed it, because 'e' is a letter.
    for stray in ('1.316e+03', '2.000e-03', '0.000e+00', '2941.0', '  1190 '):
        assert looks_numeric(stray), stray
    for real in ('Hantek,DSO2C50,CN123456,1.0.8', 'RIGOL TECHNOLOGIES,DS1054Z,,00.04'):
        assert not looks_numeric(real), real
    assert not looks_numeric('') and not looks_numeric(None)
    # ---- fire-detect on FREQuency: idle measures NO frequency, keyed measures one ----
    # Bench data, DSO2C50 fw 1.0.8, 2026-09-10, CH2, 8 runs of 5 polls:
    #   not firing -> 0.0 every time        firing -> 1351 / 1786 / 1852 / 1923 / 2273 / 2941 Hz
    # (Those keyed numbers are aliased — the detector sits at the envelope timebase where a
    # 0.3-4 MHz carrier is ~80x undersampled. The value is nonsense; its existence is not.)
    f_fire, f_rel = fire_test('FREQuency', 0.0, 100.0)
    for idle in (0.0,):
        assert not f_fire(idle) and f_rel(idle)            # idle: no fire, counts as released
    for keyed in (1351.0, 1786.0, 1852.0, 1923.0, 2273.0, 2941.0):
        assert f_fire(keyed) and not f_rel(keyed), keyed   # every keyed reading observed
    assert not f_fire(None) and not f_rel(None)            # a dropped query is NOT 'idle'
    # a stale VPP reply leaking into the queue is volts, not kilohertz — must not read as fire
    for orphan in (0.002, 0.05, 1.19, 2.5):
        assert not f_fire(orphan), orphan
    # ---- the VPP path stays as a fallback and keeps its own logic ----
    v_fire, v_rel = fire_test('VPP', 0.008)
    assert v_fire(0.0537) and not v_fire(0.002)            # 2 W burst trips, 1 ADC code does not
    assert v_rel(0.002) and not v_rel(0.0537) and not v_rel(None)
    # ---- fire-detect: threshold from MEASURED IDLE, sanity bound from PHYSICS ----
    # Neither asks the scope for a range. Querying one between :MEASure polls is what desynced
    # the reply buffer and reported CH2 at 4000 V/div beside a 0.002 V Vpp on the same channel.
    import statistics as _st
    # the median is the whole point: one bad reply in four must not set the trigger level
    assert _st.median([0.002, 0.002, 0.002, 0.05]) == 0.002
    assert max([0.002, 0.002, 0.002, 0.05]) == 0.05      # what the old rule would have used
    thr_med, thr_max = 0.002 * 4, 0.05 * 4
    burst2W = 2 * math.sqrt(2) * math.sqrt(2.0 / 500) * COIL_V_PER_A * 3   # 2 W, 500 ohm, 3 turns
    assert burst2W > thr_med, (burst2W, thr_med)         # median: the smoke test still trips it
    assert burst2W < thr_max, (burst2W, thr_max)         # max: it would have been missed
    # the bound is the rig, not the screen — and it kills the 4000 V reply by a wide margin
    b = vpp_bound(1000, 500, 3)
    assert 4000 > b * 100, b
    assert b > 2 * math.sqrt(2) * math.sqrt(400.0 / 500) * COIL_V_PER_A * 3   # 400 W still passes
    assert vpp_bound(1000, 500, 6) == 2 * vpp_bound(1000, 500, 3)            # scales with turns
    assert abs(vpp_bound(1000, 200, 1) / vpp_bound(1000, 500, 1) - math.sqrt(500 / 200)) < 1e-9
    assert vpp_ceiling(0.05) > screen_pp(0.05)           # ADC holds more than the screen shows
    assert abs(vpp_ceiling(0.05) / screen_pp(0.05) - 1.27) < 1e-9
    # ---- a :MEASure reply the ADC cannot physically have produced must be thrown away ----
    # The real one: an idle CH2 at 50 mV/div came back as 4000 V, which set the fire-detect
    # threshold to 12 kV (~8e11 W into 500 ohm) and armed a detector nothing could ever trip.
    assert abs(vpp_ceiling(0.05) - 0.508) < 1e-9          # +/-127 codes at 25 codes/div
    assert 4000.0 > vpp_ceiling(0.05) * 1.05              # ...so that reply is rejected
    assert 0.5 <= vpp_ceiling(0.05) * 1.05                # a genuinely railed frame still passes
    assert vpp_ceiling(2.0) > vpp_ceiling(0.05)           # the bound tracks the range
    # ---- range headroom: back off N 1-2-5 steps, never off the end of the range ----
    assert _gear_step(0.05, 2) == 0.2 and _gear_step(0.05, -1) == 0.02
    assert _gear_step(0.06, 1) == 0.1                      # snaps to the nearest real gear first
    assert _gear_step(GEARS[-1], 3) == GEARS[-1] and _gear_step(GEARS[0], -3) == GEARS[0]   # clamped
    assert _gear_step(_gear_step(0.05, 2), -2) == 0.05     # a burst's autoscale can undo the headroom
    # ---- machine profiles: the two ways an OEM writes a spec must land on the SAME band ----
    # Ellman prints "25-37 W"; Valleylab prints "31 W +/- 19.35%". Identical window, so both
    # entry styles must store identical (expected_W, tol_pct) -- otherwise the same machine
    # grades differently depending on which manual the tech was reading.
    e1, t1 = band_from('minmax', 25, 37)
    e2, t2 = band_from('pm', 31, 100 * 6 / 31)
    assert abs(e1 - e2) < 1e-9 and abs(t1 - t2) < 1e-9, (e1, t1, e2, t2)
    assert abs(band_to('minmax', e1, t1)[0] - 25) < 1e-9 and abs(band_to('minmax', e1, t1)[1] - 37) < 1e-9
    assert band_from('minmax', 37, 25) == band_from('minmax', 25, 37)   # order must not matter
    assert band_to('pm', 31, 20) == (31, 20) and band_from('pm', 31, 20) == (31, 20)
    assert band_from('minmax', 0, 7) == (3.5, 100.0)          # Ellman's "0 to 7 W" bottom band
    assert band_from('minmax', 0, 0) == (0.0, 0.0)            # a dead setting must not divide by zero
    assert g(3.0) == 3 and g(3.5) == 3.5
    assert profile_slug('Ellman Dento-Surg 90 FFP') == 'ellman-dento-surg-90-ffp'
    # and the profile must survive the round trip to disk that the wizard's Save/Open does
    import tempfile
    _rd = globals()['refs_dirs']
    with tempfile.TemporaryDirectory() as td:
        globals()['refs_dirs'] = lambda: [td]
        try:
            save_profile('Ellman Dento-Surg 90 FFP', 'Ellman', [
                {'mode': 'fulg', 'setting': 6.0, 'load_ohm': 500.0, 'expected_W': e1, 'tol_pct': t1},
                {'mode': 'cut', 'setting': 0.0, 'load_ohm': 500.0, 'expected_W': 3.5, 'tol_pct': 100.0}])
            assert list_profiles() == ['ellman-dento-surg-90-ffp']
            mdl, back = load_profile('Ellman Dento-Surg 90 FFP')
            assert mdl == 'Ellman' and [r['mode'] for r in back] == ['cut', 'fulg']   # sorted on load
            lo, hi = band_to('minmax', back[1]['expected_W'], back[1]['tol_pct'])
            assert abs(lo - 25) < 1e-9 and abs(hi - 37) < 1e-9, (lo, hi)
            # the same file has to read back through the path --watch/--compare already use
            assert identify(ref_bands('ellman-dento-surg-90-ffp'), 'fulg', 30.0)[0] == [6]
        finally:
            globals()['refs_dirs'] = _rd
    print("demo OK — 20 W by all three methods, freq 4 kHz + fractional-bin 471 kHz, Vrms 100 V, "
          "snap125, clip/rail detection, session/verdict, min-max == +/- profile round trip")


def make_parser():
    """One source of truth for every flag and default. The wizard fills its own args
    from make_parser().parse_args([]) rather than retyping them — a duplicated default
    list is how the detector once ran without a --fire-frac attribute at all."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--wizard', action='store_true', help='calibration wizard: build/open a machine profile (refs/<machine>.csv), walk mode x setting with fire-detect, grade every point. THE DEFAULT on double-click')
    ap.add_argument('--gui', action='store_true', help='the old one-shot capture+report window')
    ap.add_argument('--session', nargs='*', metavar='NAME', help='power-sweep: connect+autoscale once, capture on each Enter, log to <NAME>_sweep.csv. Words are joined: --session ellman 1234 cut -> ellman_1234_cut_sweep.csv')
    ap.add_argument('--envelope', action='store_true', help='measure a MODULATED mode (blend/coag/fulg) with a long-window mean(v*i); run on cut first to validate. COMBINE WITH --watch for fire-detected envelope bursts')
    ap.add_argument('--envdepth', type=int, default=10000, help='memory depth for envelope captures. 10000 = a 200 ms record in ~2.9 s — the default. 200 ms is exactly 12 mains periods at 60 Hz AND 10 at 50 Hz, so the record holds a WHOLE number of envelope periods either way and there is no partial-period error (120 ms does not, and reads 1.5%% high on half-wave coag). Raise to 20000/40000 only if the thirds-disagree warning fires')
    ap.add_argument('--recarrier', action='store_true', help='re-measure the carrier at every envelope point instead of reusing the first (the carrier is a property of the machine, not the dial setting)')
    ap.add_argument('--envwin', type=float, default=50e-3, metavar='SEC', help='envelope averaging window in seconds (default 0.05 = 3 cycles of a 60 Hz envelope)')
    ap.add_argument('--compare', metavar='SWEEP.CSV', help='score a saved session against --ref and chart it (no scope needed)')
    ap.add_argument('--ref', default='ellman-surgitron-4.0', metavar='MACHINE', help='OEM reference table: a name resolved in refs/<name>.csv, or an explicit path. Columns: model,mode,setting,load_ohm,expected_W,tol_pct')
    ap.add_argument('--live', action='store_true', help='rolling live waveform window (poll+redraw; close window to stop)')
    ap.add_argument('--watch', action='store_true', help='FIRE-DETECT: wait for the ESU to key, capture automatically, log each burst, re-arm. No countdown needed.')
    ap.add_argument('--fire-ch', type=int, default=2, help='channel the fire-detector watches (default 2 = the Pearson current channel, the only quiet one)')
    ap.add_argument('--headroom', type=int, default=2, metavar='STEPS', help='--watch: back CH1/CH2 off this many 1-2-5 range steps after each burst, so walking a dial UP never clips the next reading on screen. Each measurement re-ranges, so it does not accumulate. 0 = off')
    ap.add_argument('--expect-w', type=float, default=0.0, metavar='W', dest='expect_w', help='watts this next burst is expected to produce. Both channels are then ranged for it with the pedal UP, and the keyed window is just the capture — no measure/step/re-measure. --watch fills this from the --ref table automatically; the wizard fills it from the profile')
    ap.add_argument('--crest-hint', type=float, default=3.5, metavar='X', help='peak/rms assumed when pre-ranging the FIRST burst of a mode from its expected watts (cut is 1.41, coag/fulg 4-6; 3.5 splits it). Every burst after uses the crest that mode actually measured')
    ap.add_argument('--fire-item', default='FREQuency', metavar='ITEM', help='what the fire-detector reads: FREQuency (default — an idle channel measures no frequency at all, so there is no baseline, threshold or range to get wrong) or VPP (the older level-based path, kept as a fallback)')
    ap.add_argument('--fire-min-hz', type=float, default=100.0, metavar='HZ', help='--fire-item FREQuency: a reading above this is a fire. Only there to reject a stale VPP reply (volts) leaking into the queue; keyed readings are kilohertz')
    ap.add_argument('--fire-gain', type=float, default=4.0, metavar='X', help='fire-detect trips at this multiple of the MEDIAN idle reading (default 4). Derived entirely from measured idle — no scope range query, which is what desyncs this firmware')
    ap.add_argument('--max-watts', type=float, default=1000, metavar='W', help='most any mode on this machine could deliver. Only used to reject impossible VPP replies (a 4000 V one armed the detector at 320 V once); it is a sanity bound, not a limit on what gets measured')
    ap.add_argument('--fire-threshold', type=float, default=0.0, help='detector threshold in raw channel volts. Overrides --fire-gain and the measured idle outright; 0 = auto')
    ap.add_argument('--session-name', metavar='NAME', help='--watch: log to <NAME>_bursts.csv (default esu_watch_bursts.csv)')
    ap.add_argument('--math', action='store_true', help='--watch: re-test the scope MATH VAVG read (on-scope v*i). PROVEN DEAD on fw 1.0.8 and costs ~1 s per burst — diagnostic only, leave it off')
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
    return ap


if __name__ == '__main__':
    ap = make_parser()
    a = ap.parse_args()
    if a.wizard or len(sys.argv) == 1: wizard()   # no args (double-click) -> the wizard
    elif a.gui: gui()
    elif a.compare: compare(a)
    elif a.watch: watch(a)                   # before --envelope: the two COMBINE, watch drives
    elif a.envelope: envelope(a)
    elif a.session is not None: session(a)   # [] when --session given bare -> still a session
    elif a.live: live(a)
    elif a.demo: demo()
    elif a.list: list_resources()
    elif a.calcheck: calcheck(a.resource)
    elif a.probe: probe(a.resource)
    else: run(a)
