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
python esu_test.py                                   # GUI (default on double-click)
python esu_test.py --sn ELLMAN123 --mode "cut 50W" --load 500 --setpoint 50 --turns 3
```

### Modes (pick one; default is the GUI)

| Flag | What it does | Example |
|------|--------------|---------|
| *(none)* | Launch the GUI (also on double-click) | `python esu_test.py` |
| `--gui` | Force the GUI | `python esu_test.py --gui` |
| `--session` | **Power-sweep mode** — connect + autoscale **once**, then capture on each Enter with the scope held open. Logs to `<name>_sweep.csv`. Use this for a full sweep instead of one slow process per setpoint. See [Sessions](#sessions--sweeping-a-machine) | `python esu_test.py --session ellman 1234 cut --load 500` |
| `--envelope` | **Measure a modulated mode** (blend/coag/fulg) by peak-detect envelope averaging. Run on `cut` first to validate. See [Modulated modes](#modulated-modes-blend--coag--fulg--read-this-before-trusting-a-number) | `python esu_test.py --envelope --mode cutcoag --load 500` |
| `--compare` | **Score a saved sweep against an OEM spec table** and chart it. No scope needed. See [Comparing against OEM spec](#comparing-against-oem-spec---compare) | `python esu_test.py --compare ellman_1234_cut_sweep.csv` |
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

## GUI (for techs)
Double-click the exe → fill S/N, mode, setpoint, load, **Coil turns** → **Test Connection** → **Capture & Report**.
Autoscale runs automatically. The report opens in the browser.

---

## Build a portable exe (Windows)

```
build.bat
```
Produces a single `dist\esu_test.exe` (Python + all libs + `libusb` bundled). Copy it anywhere;
each target PC still needs the one-time driver step above.

---

## Notes & limits

- **Firmware flashing is not needed** — this uses the stock DSO2000 SCPI.
- HRES/averaging need a **repetitive** signal (steady cut/coag). For a **one-shot burst**, use `--acq NORMal`.
- Deep memory is slow over USB (minutes at millions of points) — keep memory depth modest.
- Power `mean(v·i)` is noise-immune; `Irms²·R` is not — if they disagree, suspect load drift or reactance.
- The vendor SCPI manual (`DSO2000 Series SCPI Programmers Manual.pdf`) is on
  [hantek.com](https://www.hantek.com/) → product downloads. Not redistributed here.

## License
MIT — see [LICENSE](LICENSE).
