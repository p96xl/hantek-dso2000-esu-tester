# OEM reference tables — one file per machine

`--compare` grades a sweep against the machine's own service-manual numbers.
Point at one by bare name; it resolves here:

```
python esu_test.py --compare fxc_1234_sweep.csv --ref force-fxc     # -> refs/force-fxc.csv
python esu_test.py --compare sweep.csv --ref /some/other/path.csv   # explicit path also works
```

A wrong or missing name lists what's here, so a typo can't silently grade zero rows.

## Format

```csv
model,mode,setting,load_ohm,expected_W,tol_pct
Valleylab Force FX-C,cut,50,300,150,20
Valleylab Force FX-C,cut,100,300,300,20
```

| Column | Notes |
|---|---|
| `model` | Optional, but **printed in the report header**. Grading a machine against another model's table is the one mistake here that yields a confident, wrong PASS. Fill it in. |
| `mode` | Must match what the session logged. Add front-panel spellings as extra rows — see aliases below. |
| `setting` | Front-panel number. Sweep at any granularity: `expected_W` is **interpolated** between rows, so a table at 0/10/20… grades a sweep taken at 5/15/25…. Settings outside the table's range are reported as skipped, never extrapolated. |
| `load_ohm` | The load the OEM figure assumes. `--compare` warns if the logged load differs by >5%. |
| `expected_W` | From the service manual's power-vs-setting figure. |
| `tol_pct` | Per row, so you can tighten individual points. |
| `envelope` | **Optional**, `1` or `0`. `1` = capture with the long-window envelope method (modulated modes: blend / coag / fulg); `0`, an empty cell, or no column at all = plain direct capture. A mode is one or the other — you set it once per machine off the service manual, and the wizard has no runtime switch to contradict it. Nothing is auto-detected, but a `0` row whose capture turns out to straddle an envelope is flagged **⚠ MODULATED — re-read as envelope** rather than graded. |
| `wiring` | **Optional.** Free text, shown in big letters on the run banner, e.g. `Both leads -> the two BIPOLAR jacks` or `Active -> right MONOPOLAR jack; return -> REM jack`. The banner shouts **CHANGE THE LEADS** whenever it differs from the row above, so re-plugging is prompted, not remembered. |

**Include a `setting,0` row at `0 W`** — it anchors the interpolation so low settings grade correctly.

## Front-panel names ≠ manual names

Manuals and front panels often disagree. Add the panel spelling as duplicate rows with the same
numbers rather than renaming anything — costs nothing, removes a whole class of confusion.

Ellman example: the manual says Cut/Coag and Hemo, the panel says BLEND and COAG. So that
machine's table carries `cutcoag` **and** `blend`, `hemo` **and** `coag`.

## Adding a machine

1. Find the service manual's **power output vs. digital setting** figure (and note the load it
   was measured at — it's usually per-mode, and bipolar is often different).
2. Find the stated tolerance. If the manual has a note like *"shall not deviate by more than
   ±20%"*, use that — it usually covers test-equipment variation too, which is what you want.
3. Write the CSV. Filename becomes the `--ref` name: `refs/force-fxc.csv` → `--ref force-fxc`.
4. Also worth capturing while you're in the manual: the **power vs. load** curve (tells you
   whether your load is at the design match point) and any **open-circuit voltage** figure
   (a free HV-probe calibration reference at the machine's own frequency).

## Not in this repo

`refs/*.csv` is gitignored and no table ships in the exe. The numbers are transcribed from
vendor service manuals, so they stay on the bench (or a shared drive), not on GitHub. This
file is the format; write your own, or build one in the wizard and **Save…** it anywhere.
