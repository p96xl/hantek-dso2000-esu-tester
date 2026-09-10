# Hantek DSO2000 ESU Output Tester

Automated RF **power / voltage / current** test reports from a **Hantek DSO2C50** (DSO2000 series)
oscilloscope over SCPI/USBTMC — built for verifying electrosurgical unit (ESU) generator output
into a rated resistive load. Captures both channels, computes power three ways, plots the waveform,
and writes a printable HTML report. Ships a dead-simple GUI for bench techs.

Measurement rig: **Ch1** = voltage across the load (HV probe), **Ch2** = current via a **Pearson 110A**
current monitor. This is a DIY equivalent of a BC Biomedical ESU-2050 analyzer.

> The DSO2000 firmware's SCPI is a stripped, quirky Rigol clone — no `:MEASure` subsystem, no screenshot,
> and waveform export is the undocumented `PRIVate:WAVeform:DATA:ALL?` with a 128-byte-per-packet header.
> This tool works around all of that. Waveform protocol adapted from
> [phmarek/hantek-dso2000](https://github.com/phmarek/hantek-dso2000).

---

## Quick start (any Windows PC)

**Option A — prebuilt exe, NO Python needed** (recommended for techs):
1. Download `esu_test.exe` from the [**Releases**](../../releases/latest) page.
   (Python and all libraries are bundled inside — nothing to install.)
2. Do the **one-time driver step** (below).
3. Double-click it → GUI opens.

**Option B — run from source** (needs Python 3):
```
pip install -r requirements.txt
python esu_test.py            # no args -> GUI
```

### One-time driver step (required, per PC)
The scope talks USBTMC, which needs a USB driver:
1. On the scope: **Utility → I/O → USB Device = Computer / USBTMC** (not Printer).
2. Install a USB backend — pick one:
   - **Zadig (recommended, offline):** run [Zadig](https://zadig.akeo.ie/), *Options → List All Devices*,
     select the scope's **USBTMC interface**, install **WinUSB**. `esu_test.py` bundles `libusb` so nothing else is needed.
   - **NI-VISA:** install the NI-VISA runtime; the script auto-detects it.
3. Confirm it's seen: `python esu_test.py --list`

---

## Bench setup

| Channel | Connect | Scope setting |
|---------|---------|---------------|
| **Ch1** | Voltage across the load via HV probe | Set the channel **probe ratio to your probe** (e.g. 10×/1000×) — the scope reports true volts |
| **Ch2** | Pearson 110A current monitor (BNC) | Probe ratio **1×**; **DC coupling**; turn **V/div down** so the current fills several divisions |

Then edit the constants at the top of `esu_test.py`:
```python
PROBE_RATIO  = 1.0    # leave 1 if the scope's own probe setting scales CH1 voltage
COIL_V_PER_A = 0.1    # Pearson 110A: 0.1 (1 MΩ input) or 0.05 (50 Ω input)
CODES_PER_DIV = 25.0  # DSO2000 ADC codes/div — verify once with --calcheck
```

**Coupling = DC. Don't double-scale** (let either the scope's probe ratio OR `PROBE_RATIO` handle CH1, not both).
Current always needs `÷ COIL_V_PER_A` because the scope only shows volts.

---

## Usage

```
python esu_test.py                                   # calibration WIZARD (default on double-click)
python esu_test.py --sn ELLMAN123 --mode "cut 50W" --load 500 --setpoint 50 --turns 3
```

### Modes (pick one; default is the wizard)

| Flag | What it does | Example |
|------|--------------|---------|
| *(none)* | Launch the **calibration wizard** (also on double-click). See [Wizard](#the-calibration-wizard-for-techs) | `python esu_test.py` |
| `--wizard` | Force the wizard | `python esu_test.py --wizard` |
| `--gui` | The old one-shot capture-and-report window | `python esu_test.py --gui` |
| `--session` | **Power-sweep mode** — connect + autoscale **once**, then capture on each Enter with the scope held open. Logs to `<name>_sweep.csv`. Use this for a full sweep instead of one slow process per setpoint. See [Sessions](#sessions--sweeping-a-machine) | `python esu_test.py --session ellman 1234 cut --load 500` |
| `--envelope` | **Measure a modulated mode** (blend/coag/fulg) with a long-window `mean(v·i)`. Run on `cut` first to validate. **Combines with `--watch`.** See [Modulated modes](#modulated-modes-blend--coag--fulg--read-this-before-trusting-a-number) | `python esu_test.py --envelope --mode cutcoag --load 500` |
| `--compare` | **Score a saved sweep against an OEM spec table** and chart it. No scope needed. See [Comparing against OEM spec](#comparing-against-oem-spec---compare) | `python esu_test.py --compare ellman_1234_cut_sweep.csv` |
| `--watch` | **Fire-detect** — waits for the ESU to key, captures on its own, logs each burst, re-arms. No counting down against a capture. Watches the *current* channel (the only quiet one) and trips on **`:MEASure FREQuency` being nonzero at all** — no baseline, no threshold. Any `VPP` reply above what the ADC can physically produce at that range is discarded, not believed — see [the 12 kV threshold](#when-the-scope-lies-about-a-measurement). Names the candidate dial settings if a `--ref` table is loaded | `python esu_test.py --watch --load 500 --turns 3 --mode fulg --ref ellman-dento-surg-90-ffp` |
| `--watch --envelope` | The pair you actually want on a modulated mode: fire-detect **and** an honest long-window average. Burst 1 costs ~10 s (it hunts the carrier), every burst after ~6 s (the carrier is cached — it is a property of the machine, not the dial). A burst released too early is **DISCARDED**, not logged. Each burst prints how long the ESU was actually keyed | `python esu_test.py --watch --envelope --mode coag --load 500 --turns 3` |
| `--live` | **Live waveform window** — poll + redraw until you close it. Not true streaming (the DSO2000 has no streaming SCPI, only whole-frame reads → ~1–3 Hz), but enough to watch the wave change as you turn the dial | `python esu_test.py --live --load 200 --turns 3` |
| `--demo` | Self-test the math, no scope needed | `python esu_test.py --demo` |
| `--list` | List VISA resources + `*IDN?` | `python esu_test.py --list` |
| `--calcheck` | Capture + print raw scope-volts / noise / ADC codes to verify scaling | `python esu_test.py --calcheck` |
| `--probe` | Dump the waveform packet structure (reassembly diagnostic) | `python esu_test.py --probe` |

### Report options (the main capture-and-report path)

| Flag | Default | What it does | Example |
|------|---------|--------------|---------|
| `--sn` | `?` | ESU unit serial number (report header + output filename) | `--sn FORCEFXC` |
| `--mode` | `?` | Mode label. In a session it's the starting mode and is logged per row — use a name your reference table knows (`cut`, `hemo`, `bipolar`…) if you plan to `--compare` | `--mode cut` |
| `--setpoint` | `?` | Front-panel setting (for the report) | `--setpoint 50` |
| `--ref` | `esu_reference.csv` | OEM table used by `--compare`. One file per machine model | `--ref refs/bovie_a1350.csv` |
| `--load` | `500` | Load resistance in ohm — **must be the real value**, the `Vrms²/R` and `Irms²·R` powers depend on it | `--load 200` |
| `--turns` | `1` | Passes of the ESU wire through the Pearson coil. Coil reads amp-**turns**, so N turns = N× signal off the noise floor; amps are divided back by N in software | `--turns 3` |
| `--cycles` | `6` | Approx # of waveform cycles autoscale puts on screen (snaps to nearest scope timebase gear) | `--cycles 5` |
| `--no-autoscale` | off | Skip auto V/div + timebase; use the scope exactly as set | `--no-autoscale` |
| `--acq` | `HRESolution` | Acquisition type: `HRESolution` \| `AVERage` \| `PEAK` \| `NORMal` (HRES cuts noise; use `NORMal` for one-shot bursts) | `--acq AVERage` |
| `--count` | `64` | Number of averages when `--acq AVERage` | `--count 128` |
| `--smooth` | `0` | Display-only moving-average on the current plot (samples); does **not** touch the RMS/power numbers | `--smooth 10` |
| `--resource` | auto | Explicit VISA resource string (default: autodetect USB) | `--resource USB0::...::INSTR` |
| `--out` | `esu_report.html` | Output HTML path (`.png` written alongside) | `--out force_fxc.html` |

Output: `esu_report*.html` (embedded plot + measurements, **Ctrl+P → Save as PDF**) and a `.png`.
The report flags **low ADC resolution** if a channel's signal spans under ~10 codes (turn its V/div down, or `--turns` more).

## Sessions — sweeping a machine

A session holds the scope open so you can walk the dial without a reconnect + re-autoscale
per point. Name it after the machine you're testing:

```
python esu_test.py --session ellman 1234 cut --mode cut --load 500 --turns 3 --no-autoscale
```

Words after `--session` are joined into the filename, so no quoting needed:
`--session ellman 1234 cut` → **`ellman_1234_cut_sweep.csv`**.

At the prompt:

| Type | Does |
|------|------|
| `50` | Capture at dial setting 50 |
| `m hemo` | **Switch mode** — later captures are logged as `hemo` |
| `l 200` | **Change the load** in ohm (bipolar needs 200, not 500) |
| `a` | Re-autoscale |
| `q` | Quit |

**Yes, you can change modes inside one session.** Each row carries its own `mode` and
`load_ohm`, so one file per *machine* is the intended unit — sweep cut, then `m hemo`, then
`m bipolar` + `l 200`, all into `ellman_1234_sweep.csv`. `--compare` grades each mode against
its own curve and plots them as separate series. The `--mode` / `--load` flags just set the
starting values.

> Changing mode does **not** change the load. Bipolar is tested at 200 Ω while everything
> else is 500 Ω — use `l 200` when you switch, or every bipolar row is graded against a
> load you weren't using.

### Redoing a bad reading

**Named session → re-capturing a setting overwrites it.** Type `70` again and the new
reading replaces the old row *in place*, keeping your sweep order. You'll see:

```
overwrote earlier capture at cut 70 (12.3 W -> 89.6 W)
```

The dedupe key is (mode, setting), so `cut 70` and `hemo 70` don't collide. Existing files
are reread on startup, so quitting and resuming keeps overwriting instead of duplicating.

**Bare `--session` → append-only.** Every capture is kept, nothing overwritten — for
debugging, repeatability checks, or watching a reading drift as something warms up.

## Comparing against OEM spec (`--compare`)

Grades a saved sweep against a manufacturer's power table and charts it. Runs offline:

```
python esu_test.py --compare ellman_1234_cut_sweep.csv
python esu_test.py --compare bovie_a1350_sweep.csv --ref refs/bovie_a1350.csv
```

```
ellman_1234_cut_sweep.csv  vs  esu_reference.csv  [Ellman Surgitron 4.0 Dual RF/120 IEC]
mode        set   meas W   spec W           window     dev   result
cut           5     12.8     12.5   10.0-15.0       +2.8%   PASS
cut          15     31.9     35.0   28.0-42.0       -8.8%   PASS
cut          50     74.6     87.0   69.6-104.4     -14.3%   PASS
cut         100    106.1    123.0   98.4-147.6     -13.8%   PASS

20/20 within tolerance
Chart -> ellman_1234_cut_sweep_vs_spec.png
```

The chart draws the spec curve, a shaded ±tolerance band, and your points — circles for
pass, **X** for fail.

### The reference table — one file per machine in `refs/`

`--ref` takes a **bare machine name**, resolved as `refs/<name>.csv`, or an explicit path:

```
python esu_test.py --compare fxc_1234_sweep.csv --ref force-fxc
python esu_test.py --compare blend_sweep.csv                      # default: ellman-surgitron-4.0
```

A name that doesn't exist lists what does, so a typo can't silently grade zero rows.

```csv
model,mode,setting,load_ohm,expected_W,tol_pct
Ellman Surgitron 4.0 Dual RF/120 IEC,cut,50,500,87,20
Ellman Surgitron 4.0 Dual RF/120 IEC,bipolar,50,200,87,20      # note the 200 ohm
```

Format, how to add a machine, and the front-panel-vs-manual naming trap: **`refs/README.md`**.

**Sweep at any granularity.** `expected_W` is *interpolated* along the curve, so settings at
5, 15, 45 grade fine against a table listing only 0, 10, 20… Tolerance and load interpolate too.
Settings past the ends of the table are **reported as skipped, never extrapolated**:

```
SKIPPED 2 row(s) not covered by refs/ellman-surgitron-4.0.csv: modes=['cut'], settings -5-120
reference covers: bipolar 0-100, blend 0-100, coag 0-100, cut 0-100, cutcoag 0-100, fulg 0-100, hemo 0-100
```

A mode name that isn't in the reference skips loudly too.

### What gets graded

`P_meanVI` — `mean(v·i)`. It's the only estimate that neither divides by your typed load nor
assumes the load is purely resistive, so it stays right when the other two drift. Sweeps
written before this feature have no `mode` column; pass `--mode cut` and it fills it in.

## Modulated modes (blend / coag / fulg) — read this before trusting a number

Only **CUT** is continuous wave. Every other mode is a 4 MHz carrier under an envelope, and a
capture sized to show the *carrier* is far shorter than one *envelope* period. You then measure
whichever slice of the envelope the trigger landed on. Same simulated signal (true average
**45.0 W**), four trigger positions:

```
window at  0.5 ms ->   6.25 W    (14% of true)
window at  4.0 ms -> 176.77 W   (393% of true)
window at  8.3 ms ->   0.03 W     (0% of true)
window at 12.0 ms ->   0.00 W     (0% of true)
```

That's the "bouncy, all over the place" reading. The tool now warns when it detects this:

```
WARNING: envelope varies 64% across this capture -- the window is shorter than the
         modulation period. Vrms and power are whatever the trigger landed on, NOT
         average power.
```

**A frequency readout that isn't ~4 MHz is the same symptom.** All Ellman modes use a 4.0 MHz
carrier (§3.1); `f=989100Hz` means the counter is chewing on a gated carrier, not that the
carrier moved.

### Three things that look like fixes and aren't

| Tempting | Why it fails |
|----------|--------------|
| `--acq HRESolution` | HRES boxcar-averages raw samples per point. At a slow timebase that low-passes the 4 MHz carrier to nothing — measured **0.00% of true** in simulation. Use `--acq NORMal`. |
| Just slow the timebase | 4.000 MHz divides *exactly* into every 1-2-5 scope rate (÷32 at 125 kSa/s, ÷16 at 250 k, ÷8 at 500 k, ÷4 at 1 M). You sample the identical carrier phase every time — coherent sampling, answer can be anything including zero. |
| Average many captures | Converges, but slowly: 500 frames still scatters **±7%**. Not worth 500 USB transfers. |

### What actually works — `--envelope`

**One long window, plain `mean(v·i)`.** V and I are sampled *simultaneously*, so `v(t)·i(t)` is
correct at every instant no matter how badly the 4 MHz carrier is aliased. Average over a window
spanning many modulation periods and the carrier phase averages out along with the envelope.
No peak detect, no `/2`, no phase correction.

```
python esu_test.py --envelope --mode cut --load 500 --turns 3      # validate first
python esu_test.py --envelope --mode cutcoag --load 500 --turns 3

  in-burst carrier: 3,999,533 Hz, phase 3 deg
  long window: 80.0 ms, 50 kSa/s, acq=NORMal, 319,963 carrier cycles averaged
  P (mean v*i, long window) = 43.1 | 43.4 | 43.2  ->  43.2 W   (spread 0.3%)
```

**Speed.** One deep capture replaces the three shallow ones this used to take, the carrier is
measured once per machine rather than once per setpoint, and V/div is re-ranged only when
clipping is actually detected instead of before every reading. Measured on a DSO2C50:
`autoscale` alone costs **12.8 s** and used to run twice per reading. Net **~34 s → ~5.5 s per
sweep point** (the first point still pays ~20 s for the carrier hunt), on 3.3× more envelope
data — 800 ms of record instead of 3 × 80 ms. `--envdepth` sets the depth, `--recarrier` forces
the carrier to be re-measured.

Accuracy vs known truth (simulated at 50 and 125 kSa/s, with a 3,999,533 Hz carrier and 13° stray C):

| Mode | Envelope | Error |
|---|---|---|
| cut | CW | +0.1% |
| blend (`cutcoag`) | full-wave, 120 Hz | +0.1% |
| coag (`hemo`) | half-wave, 60 Hz | −0.0% |
| fulg | 10% square burst, 400 Hz | +0.1% to +1.6% |

**It runs 3 captures and reports the spread.** Agreement is the only honest validation — the one
real failure mode is *coherent sampling* (sample rate dividing the carrier exactly, so every
sample lands at the same phase), and repeatability is what catches it. Over 5% spread warns.

**Validate on CUT first.** The method is exact on CW, so on cut it must reproduce your normal
reading. If it does, it's trustworthy on blend/coag on *your* bench — that's the argument that
makes a report defensible.

> ⚠️ **`:ACQuire:TYPE PEAK` is NOT honoured by fw 1.0.8.** An earlier version of this feature
> assumed peak detect worked and reconstructed the envelope from it. The scope silently returned
> ordinary decimated samples, so it read **exactly half** on CW (`mean(|V|·|I|)/2` where
> `mean(|V||I|) == mean(V·I)` for in-phase sines) and reported a fake ~600 Hz "envelope" that was
> really the carrier's alias beat. The tell was **duty ≈ 0.5 on a CW signal**, which is just the
> sine statistic `mean(V²)/Vpk² = 0.5`. Acquisition type and timebase are now both read back.

### The brute-force alternative

1. **Find the envelope period first.** `--acq PEAK`, slow timebase, watch how fast the envelope
   repeats. You can't size the window without this number, and it differs per mode.
2. **Size one capture to span ≥3 envelope periods** while keeping the sample rate ≥10× the
   carrier (≥40 MSa/s). That means raising `:ACQuire:POINts` — deep memory is the only way to
   get both, and the transfer is slow.
3. **Verify by repeating.** Capture 5×. If the numbers agree within a few %, the window is long
   enough. If they scatter, it isn't — nothing else you do to the math will fix that.

Until a mode passes step 3, don't log its numbers as a measurement. **CUT is unaffected** — it's
CW, so there's no envelope to straddle.

## The trust rule — read the three power numbers
Every report computes power three independent ways: `Vrms²/R`, `Irms²·R`, and `mean(v·i)`.
**When all three agree, the scaling is correct and the number is real.** When they diverge,
one channel is mis-scaled and the divergence tells you which:
- `Irms²·R` way off, others low → **CH1 voltage probe ratio wrong** (e.g. a ×10 probe left on ×1 → voltage reads 10× low). Fix it on the scope; the tool trusts the scope's own probe ratio.
- `Vrms²/R` vs `Irms²·R` disagree → the load `R` isn't its nominal value (heating/drift) or there's reactance.
- Clean voltage + near-zero current → the current loop is **open** (or both conductors pass through the coil and cancel).

## Finding the right load
ESU power peaks at a rated load and rolls off both sides (too low = current-limited, too high = voltage-limited).
Sweep `--load` and watch `P (mean v·i)` climb toward the dial; the peak is the mode's rated impedance.
(Force FX-C bipolar Standard: ~100–200 Ω works; 50 Ω and 500 Ω both under-deliver.)

## The calibration wizard (for techs)

Double-click the exe. Two tabs, no command line.

### Tab 1 — Machine profile
A profile is just a `refs/<machine>.csv` — the **same file** `--compare` and `--watch --ref` already read. Build one once per machine model, reuse it forever.

1. Type a machine name (or pick a saved one) → **Open**, or start empty.
2. **Add a mode**: name it, give the setting range, give the load. Ellman Dento-Surg is four calls: `cut 0–10 @500Ω`, `cutcoag 0–10`, `coag 0–10`, `fulg 0–6`. A Force FX-C would be the same at `300Ω`.
3. Pick the **spec style your manual uses** — the wizard takes either:
   - **min – max** (Ellman prints "25 – 37 W")
   - **nominal ± %** (Valleylab prints "31 W ± 19.4%")

   Both store the identical band, so the same machine grades the same either way — asserted in `--demo`. Flip the radio button to *read back* a table in the other style.
4. Select the row(s) a spec applies to, type the two numbers, **Apply**. Select several rows at once when a run of settings shares a band.
5. **Save** → `refs/<machine>.csv`, next to the exe. It reopens instantly next time.

`refs/ellman-dento-surg-90-ffp.csv` already ships with the exe, so that machine needs no setup.

### Tab 2 — Run
Fill S/N, coil turns, tap multiplier. Leave **Envelope** ticked for anything but pure cut.

**Start** arms the fire-detector (~8 s of silence to baseline — *do not fire*). Then it is hands-free, and the **banner tells you what to do with the pedal by colour** — readable from the other table:

| Banner | Do this |
|--------|---------|
| 🟨 **BASELINING — DO NOT FIRE** | hands off, ~8 s |
| 🟩 **▶ PRESS THE PEDAL** | set the dial to what the banner shows, then key it |
| 🟦 **■ HOLD IT — CAPTURING** | keep it keyed, don't touch anything |
| 🟥 **✋ RELEASE THE PEDAL** | let go; it re-arms on its own |

Under that, in the biggest type on the screen: **CUT · LEVEL 3**, and the expected band beneath it.

- The row fills in with measured watts and **PASS/FAIL vs the profile**, green or red, and it aims at the next point on its own.
- **ReRead selected** → clears that row, aims back at it, **and re-ranges the scope for it immediately**. Jump from setting 9 back to setting 2 and the vertical scale follows before you touch the footswitch, rather than staying on setting 9's range until the next burst.
- **Re-run whole mode** → clears every row of that mode and starts it over.
- **Force re-arm** → escape hatch if it thinks the pedal is still down when it isn't.
- **Save results CSV** → `<machine>_<sn>_results.csv` with the measured value, the spec, and the pass/fail beside each other.

### When the scope lies about a measurement
`refs/dso2c50-scpi-commands.md` records that fw 1.0.8 returns a *frequency* for `VRMS` (16670 / 25000 / 5556 against real volts) and desyncs its `:MEASure` reply buffer when other queries are interleaved with it. That is not a curiosity — one 4000 V "idle" reading on CH2 set the fire-detect threshold to **12 kV, ~8×10¹¹ W into 500 Ω**, and armed a detector nothing could ever trip.

**The root cause, found 2026-09-10:** a `:MEASure` query that times out is *not cancelled*. The scope still produces that reply — it just arrives after `scope_meas` stopped listening, and the **next** query reads it instead of its own answer. One timeout poisons everything after it. And the queue outlives the process: a fresh run's `*IDN?` came back as `2.000e-03` (the previous run's orphaned CH2 Vpp) and `1.190e+03` (an orphaned `FREQuency`).

That is where the 4000 V/div, the 12 kV arm threshold and the 320 V one all came from — not from three separate bugs.

The fix is **`drain()` (USBTMC Device Clear) on every timeout**, plus a *deep* drain at `connect()` — Device Clear alone, then read off and discard whatever arrives — before an `*IDN?` that is also validated — an IDN has letters in it, a stale measurement doesn't. `scope_meas`'s timeout went 2 s → 3 s so there are fewer timeouts to clean up after.

**If the link ever wedges**, `python esu_test.py --list` is the unstick command — connecting now deep-drains before it does anything else.

Four rules follow, and all are enforced in code:

- **Drop the orphan immediately.** Any failed `:MEASure` calls `drain()` before returning `None`.
- **Never mix text and binary in the queue.** `capture()` reads every scale and offset it needs *before* the waveform block read, drains, transfers, then drains again before the next text query. A `.query()` issued after a block read decodes the leftover samples and dies with `'ascii' codec can't decode byte 0x80 in position 94` — 0x80 is a waveform sample, not text.
- **A half-delivered frame is restartable, not fatal.** Device Clear does not always abort a block transfer the scope has already begun, so a process that dies mid-waveform leaves the rest of that frame queued for whoever connects next. `capture()` detects joining at a nonzero offset (the channel-enable header rides only on the `off == 0` packet), deep-drains, and restarts the frame — up to 3 times, then says plainly that the USB link needs replugging. Before this it unpacked a `None` header and reported `a bytes-like object is required, not 'NoneType'`.
- **A bare number is never an identity.** `looks_numeric()` parses the `*IDN?` reply as a float; if it parses, it's a stray measurement and the queue gets cleared and re-asked, up to four times. Testing `any(c.isalpha())` does not work — `1.316e+03` contains a letter.

- **Never alternate `:MEASure` with ordinary queries.** Batch all the measurements, `scope.clear()`, then the plain queries. `range_by_vpp` does exactly this; the detector does no plain queries at all. Both `--watch` and the wizard clear the buffer once before polling starts.
- **The tell is internal inconsistency.** CH2 reporting 4000 V/div *while measuring 0.002 V Vpp on the same channel* is impossible — 4000 is the answer to `:ACQuire:POINts?`, handed back late. If two readings can't both be true, one is stale.
- **A reply physics rules out is thrown away.** `level()` returns `None` (not `0.0`, so it can't read as "pedal released" either).

### The detector asks the scope one question: is there a frequency?
An idle channel measures **no frequency at all**. Bench-verified on the DSO2C50, 2026-09-10, CH2, 8 runs of 5 polls:

| | reading |
|---|---|
| not firing | `0.0`, every time |
| firing | 1351 / 1786 / 1852 / 1923 / 2273 / 2941 Hz, every time |

Those keyed numbers are aliased nonsense — the detector sits at the envelope timebase, where a 0.3–4 MHz carrier is ~80× undersampled. But the *value* was never the question. **Existence is.**

So there is no baseline to sit through, no threshold to derive, no idle floor to measure, no V/div to know — and nothing left that a desynced reply can corrupt into a 12 kV trip level. Every one of those bugs lived in machinery that only existed to turn a voltage into a yes/no. This is strictly less code doing a strictly more reliable job.

Two guards remain, both cheap:

- **`--fire-min-hz` (default 100).** Only there to reject a stale VPP reply leaking into the queue — orphans are volts (0.002 … a few), keyed readings are kilohertz, and the two never overlap.
- **`None` is never "idle".** A dropped query means *unknown*; the release wait needs **two consecutive** genuine `0.0` readings, so one timeout can't re-arm mid-burst.

`--fire-item VPP` puts the old level-based path back (median idle × `--fire-gain`, with a physics bound from `--max-watts`/`--load`/`--turns`) if `FREQuency` ever misbehaves on a different scope. `--demo` asserts both paths against the eight real readings above.

### Keeping the burst short
Every second in a burst is a second the ESU is dumping into the load, and a hot heatsink is measuring a different machine than a cold one.

**The ranging that never happens is the fastest ranging.** Measure-step-remeasure costs 2–5 s with the pedal *down*, and it is solving for a number already sitting in the profile: the wizard is testing a named mode at a named setting whose expected output it knows. So the ranges are computed and written **while the pedal is still up**:

```
Vpeak = crest · √(P·R) / vmult          ->  CH1 V/div, peak at 3 of 8 divisions
Ipeak = crest · √(P/R) · 0.1 · turns    ->  CH2 V/div
```

No query, no iteration. `--watch` fills the expected watts from the `--ref` table, the wizard from the profile, or `--expect-w` sets it by hand. Being wrong is cheap and self-correcting — `clip_warn` catches a railed frame and re-ranges the old way for that one burst.

`crest` (peak/rms) is the only guess: cut is 1.41, coag and fulg run 4–6, so `--crest-hint` (3.5) covers the **first** burst of a mode and every burst after uses the crest that mode actually measured. It converges after one reading.

That takes the keyed window to roughly:

| | before | after |
|---|---|---|
| first burst of a machine | ~10 s | ~5.5 s (still hunts the carrier) |
| every burst after | ~6 s | **~3.5 s** — just the capture |

Three further things keep it down:

- **Ranging is done with `:MEASure:VPP`, not a waveform capture.** `autoscale()` pulls a full 4K frame per pass (~2.3 s each) just to compute a peak. VPP is one query (~1 s), and it's the one `:MEASure` item proven alive on fw 1.0.8 — the fire-detector already runs on it. Read on the long envelope window it's also *more* correct than the old fast pass, which ranged off whatever point of the envelope a 14 µs slice happened to land on.
- **`--envdepth` defaults to 10000 — a 200 ms record, ~2.9 s.** 200 ms is not arbitrary: it's exactly **12 mains periods at 60 Hz and 10 at 50 Hz**, so a mains-locked envelope fits a whole number of periods either way and there's no partial-period error. That's why 200 ms is exact and 120 ms isn't — 120 ms is 7.2 periods at 60 Hz and reads ~1.5% high on half-wave coag with the thirds disagreeing 16%. `--demo` checks 800/400/200 ms against a 4-second reference for both full-wave blend and half-wave coag, and checks that 120 ms *does* trip the warning.
- **The memory depth isn't restored between bursts.** Writing `:ACQuire:POINts` costs a 0.6 s settle, and paying it on every burst means paying it with the pedal down. It's put back once when the run ends.

### Ranging while you walk a dial up
A range that fitted setting 3 clips at setting 4 — a 0 → 10 walk only ever goes up. Two things handle it:

- **Every burst re-ranges before it measures.** Only the carrier *frequency* is cached between bursts (it's a property of the machine); V/div is a property of the dial setting and is re-measured each time.
- **`Headroom steps` (default 2, `--headroom` on the CLI)** backs the *voltage* channel off two 1-2-5 range steps after each burst, so the next press doesn't slam into the top of the screen before the autoscale runs. It can't accumulate, because the next measurement re-ranges anyway.

The **current** channel deliberately gets *no* headroom. It's the fire-detector, and it trips on ~3 ADC codes: at 50 mV/div that's ~1.8 W into 500 Ω, but two steps coarser it'd be ~29 W and every low setting would sail past unnoticed. So CH2 *will* look clipped on the scope when you key a high setting — that's the trigger, not the measurement, and the real capture re-ranges before it reads anything.

Every warning the CLI prints (clipping, envelope-window too short, coherent sampling) shows up in the black log pane — the exe has no console, so nothing is lost.

---

## Build a portable exe (Windows)

```
build.bat
```
Produces a single `dist\esu_test.exe` (Python + all libs + `libusb` + the shipped `refs\*.csv`
bundled). Copy it anywhere; each target PC still needs the one-time driver step above.
Machine profiles the wizard saves land in a `refs\` folder **next to the exe**, not inside it.

**`build.bat` is the only source of truth for the build.** `esu_test.spec` is *regenerated* by
it on every run and is gitignored — editing the spec has no effect.

It fails loudly now, which it did not before:

- **`--noconfirm`.** PyInstaller prompts `output directory ... will be REMOVED! Continue? (y/N)`
  whenever `dist\` already exists, and defaults to **N**. The build aborted and the script
  printed `Done.` anyway, so every rebuild after the first silently did nothing.
- **The old exe is deleted first**, so a failed build can't leave the previous one looking fresh.
- **`python -m PyInstaller`**, not the bare `pyinstaller` console script, which isn't always on
  PATH even when installed.
- **Every step checks `errorlevel`** and stops with a specific message, in parenthesised blocks —
  `if errorlevel 1 echo x & exit /b 1` is a trap, because cmd splits at `&` and runs the `exit`
  whether the `if` fired or not.
- **CRLF line endings.** cmd.exe mis-parses multi-line `if (...)` blocks in a LF-only `.bat`.

If it still produces nothing, the usual cause is Windows locking a running `esu_test.exe` —
close it and build again. The script now says so explicitly.

---

## Notes & limits

- **Firmware flashing is not needed** — this uses the stock DSO2000 SCPI.
- HRES/averaging need a **repetitive** signal (steady cut/coag). For a **one-shot burst**, use `--acq NORMal`.
- Memory depth is `:ACQuire:POINts` and it is **settable** (4K/40K/400K/4M/8M). **40K is the
  sweet spot and the 2-channel ceiling**: 5.5 s for an 800 ms record, against 2.1 s for 80 ms
  at 4K — the sample rate does not drop, the record just gets 10× longer, which is exactly
  what a modulated mode needs. Millions of points really are minutes over USB; `--envdepth`
  defaults to 40000. (`400000` silently reads back as `40000` in 2-channel mode.)
- Power `mean(v·i)` is noise-immune; `Irms²·R` is not — if they disagree, suspect load drift or reactance.
- The vendor SCPI manual (`DSO2000 Series SCPI Programmers Manual.pdf`) is on
  [hantek.com](https://www.hantek.com/) → product downloads. Not redistributed here.
- **What the firmware actually supports** — which SCPI commands work, which answer with a
  wrong number, and which are dead — is documented in
  [`refs/dso2c50-scpi-commands.md`](refs/dso2c50-scpi-commands.md), probed against a real
  DSO2C50 on fw 1.0.8 (94 of 101 commands respond). Read it before adding any SCPI.
- ⚠ **Never read `:MEASure:…:ITEM? VRMS`** — on fw 1.0.8 it returns a *frequency*, not volts.
  Compute Vrms from the trace. Likewise there is **no on-scope V·I**: `:MATH` multiply engages
  but the MATH trace is never exported and `:MEASure:MATH:ITEM?` reads zero on live signal.
- **Linux/WSL works** — the scope is USBTMC over libusb, no `usbtmc` kernel module needed. See
  the same reference for the udev rule, the `usbipd` steps, and the USBTMC recovery sequence
  (clear the halt on **Bulk-IN**, not Bulk-OUT).

## License
MIT — see [LICENSE](LICENSE).
