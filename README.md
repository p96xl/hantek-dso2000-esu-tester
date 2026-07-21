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
| `--session` | **Power-sweep mode** — connect + autoscale **once**, then capture on each Enter with the scope held open. Logs every shot to `<out>_sweep.csv`. Use this for a 1→70 W sweep instead of one slow process per setpoint | `python esu_test.py --session --load 200 --turns 3` |
| `--live` | **Live waveform window** — poll + redraw until you close it. Not true streaming (the DSO2000 has no streaming SCPI, only whole-frame reads → ~1–3 Hz), but enough to watch the wave change as you turn the dial | `python esu_test.py --live --load 200 --turns 3` |
| `--demo` | Self-test the math, no scope needed | `python esu_test.py --demo` |
| `--list` | List VISA resources + `*IDN?` | `python esu_test.py --list` |
| `--calcheck` | Capture + print raw scope-volts / noise / ADC codes to verify scaling | `python esu_test.py --calcheck` |
| `--probe` | Dump the waveform packet structure (reassembly diagnostic) | `python esu_test.py --probe` |

### Report options (the main capture-and-report path)

| Flag | Default | What it does | Example |
|------|---------|--------------|---------|
| `--sn` | `?` | ESU unit serial number (report header + output filename) | `--sn FORCEFXC` |
| `--mode` | `?` | Free-text mode label | `--mode "bipolar std 30W"` |
| `--setpoint` | `?` | Front-panel watts (for the report) | `--setpoint 30` |
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

### The trust rule — read the three power numbers
Every report computes power three independent ways: `Vrms²/R`, `Irms²·R`, and `mean(v·i)`.
**When all three agree, the scaling is correct and the number is real.** When they diverge,
one channel is mis-scaled and the divergence tells you which:
- `Irms²·R` way off, others low → **CH1 voltage probe ratio wrong** (e.g. a ×10 probe left on ×1 → voltage reads 10× low). Fix it on the scope; the tool trusts the scope's own probe ratio.
- `Vrms²/R` vs `Irms²·R` disagree → the load `R` isn't its nominal value (heating/drift) or there's reactance.
- Clean voltage + near-zero current → the current loop is **open** (or both conductors pass through the coil and cancel).

### Finding the right load
ESU power peaks at a rated load and rolls off both sides (too low = current-limited, too high = voltage-limited).
Sweep `--load` and watch `P (mean v·i)` climb toward the dial; the peak is the mode's rated impedance.
(Force FX-C bipolar Standard: ~100–200 Ω works; 50 Ω and 500 Ω both under-deliver.)

### GUI (for techs)
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
