# DSO2C50 (DSO2000 series) SCPI — what actually works on firmware 1.0.8

Probed 2026-09-09 against a real DSO2C50, `1.0.8(251205.00)`. The command list came from
the official *DSO2000 Series SCPI Programmers Manual* (196 documented query forms); a
curated, value-ordered subset was then run against the instrument with a `*IDN?` sentinel
after every command. **94 of 101 responded.** Raw log: `dso2c50-scpi-probe.txt`.

> Values below were read on an idle scope (a DC level on CH1, CH2 open). They show that a
> command *answers*, not that the number is meaningful.

---

## The three that change how you use this tool

### `:ACQuire:POINts <n>` — memory depth. **The single biggest win.**

The manual (§3.1) documents this as **set or query**; the tool had only ever read it.
Accepted values: `4000 / 40000 / 400000 / 4000000 / 8000000`.

| depth | capture time | samples/ch | record window |
|---|---|---|---|
| 4000 | 2.1 s | 4,000 | 80 ms |
| **40000** | 5.5 s | 40,000 | **800 ms** |

**The sample rate does not drop — the record just gets 10× longer.** That is exactly what a
modulated-mode (blend / coag / fulg) measurement needs: at a 120 Hz envelope this goes from
~10 modulation periods to ~96. `--envdepth` drives it; 40000 is the ceiling in 2-channel
mode (`400000` reads back as `40000`).

⚠ `:ACQuire:DEPMem` does **not** exist. That was a wrong guess, not a firmware gap.

### `:TRIGger:SWEep AUTO|NORMal` — was a front-panel step

Set **and** read back. This is the fix for the `#9000000000` empty-block failure (a 0-byte
waveform means no frame was acquired): in AUTO the scope always fills memory. It used to be
a thing the tech had to remember. `:TRIGger:STATus?` also answers (`NOTRIG` / `AUTO` / …),
so an empty capture can now be **diagnosed** rather than guessed at.

### `:CHANnel<n>:COUPling DC|AC` — also a front-panel step

Set and read back. DC is mandatory for ESU work (AC coupling tilts low-frequency and
square-ish waveforms). Now verifiable in software instead of assumed.

---

## Working, by subsystem

| Command | Example reply | Why it matters |
|---|---|---|
| `*IDN?` | `undefined, DSO2C50, CN…, 1.0.8(251205.00)` | identity; also the liveness sentinel |
| `*OPC?` / `*ESR?` / `*STB?` | `1` / `32` / `4` | standard status |
| `:SYSTem:ERRor?` | `-113` | `-113` = undefined header. **Safe to interleave** |
| `:ACQuire:POINts?` | `4000` | memory depth — see above |
| `:ACQuire:SRATe?` | `50000` | confirm GSa/s vs kSa/s before trusting a frequency |
| `:ACQuire:TYPE?` | `NORMal` | `NORMal｜AVERage｜PEAK｜HRESolution` |
| `:ACQuire:COUNt?` | `4` | averages when TYPE=AVERage |
| `:TRIGger:STATus?` | `NOTRIG` | did a frame actually acquire? |
| `:TRIGger:SWEep?` | `AUTO` | see above |
| `:TRIGger:MODe?` | `EDGE` | |
| `:TRIGger:EDGe:SOURce?/SLOPe?/LEVel?` | `CHANnel1` / `RISIng` / `0.000000e+00` | |
| `:TRIGger:HOLDoff?` / `:TRIGger:COUPling?` | `0.000001` / `DC` | |
| `:CHANnel<n>:SCALe?` / `:OFFSet?` | `1.000e+02` / `0` | volts/div + offset — the byte→volt maths |
| `:CHANnel<n>:PROBe?` | `1.000e+02` | ⚠ **read this.** A wrong probe ratio silently scales every voltage |
| `:CHANnel<n>:COUPling?` | `DC` | see above |
| `:CHANnel<n>:DISPlay?` | `1` | channel on/off |
| `:CHANnel<n>:BWLimit?` | `20M` | 20 MHz limit; the Pearson caps there anyway |
| `:CHANnel<n>:INVert?` / `:VERNier?` | `0` / `OFF` | |
| `:TIMebase:SCALe?` | `5.000000e-03` | s/div |
| `:TIMebase:POSition?` | `1.620000e-04` | ⚠ horizontal position. **NOT `:TIMebase:OFFSet`** |
| `:TIMebase:MODe?` | `MAIN` | |
| `:TIMebase:WINDow:ENABle?/SCALe?/POSition?` | `OFF` / `5.000e-05` / `0` | the zoom window |
| `:MEASure:CHANnel<n>:ITEM? <type>` | `VPP` → `4.000e+00` | **all 33 item types answer** — see caveats |
| `:MEASure:ENABle?` / `:SOURce?` / `:ADISplay?` | `ON` / `CHANnel1` / `OFF` | |
| `:CURSor:MODE?`, `:CURSor:MANual:TYPE?/SOURce?/AXValue?/AYValue?`, `:CURSor:TRACk:SOURcea?` | `OFF`, `Y`, `MATH`, … | on-screen cursors, placeable in software. Unused today |
| `:MATH:DISPlay?` / `:OPERator?` / `:SCALe?` / `:OFFSet?` | `OFF` / `ADD` / `1.000e-05` / `0` | the multiply engages, but see caveats |
| `:MATH:FFT:SOURce?` / `:WINDow?` | `CHANnel1` / `HANNing` | **there is an on-board FFT** — interesting for ESU harmonic content. Unused today |
| `:DISPlay:TYPE?/GRID?/GBRightness?/WBRightness?` | `VECTors` / `REAL` / `60` / `100` | |
| `:CALibrate:STATus?` | `1` | |
| `:SYSTem:PON?` | `DEFault` | power-on state |
| `:WAVeform:XINCrement?` | `2e-05` | seconds/sample — the tool's `dt` |
| `:WAVeform:FORMat?` / `:SOURce?` | `WORD` / `CHANnel1` | moot while the `PRIVate:` export is used |
| `PRIVate:WAVeform:DATA:ALL?` | `#9` block | **the only working waveform export.** See the project notes for the packet layout |

## ⚠ Answers, but the number is WRONG — do not use

| Command | What it really does |
|---|---|
| `:MEASure:…:ITEM? VRMS` | **Returns a FREQUENCY, not volts.** Proven on live RF: computed 94.8 / 116.9 / 75.3 / 74.6 V against replies of 16670 / 25000 / 5556 / 10000, and one burst returned VRMS *exactly equal* to FREQ (5556). Compute Vrms from the trace. |
| `:MEASure:…:ITEM? FREQuency` | Real, but **aliased to nonsense at any envelope timebase** — a ~4 MHz carrier at 50 kSa/s is ~80× undersampled. Any "frequency" under ~100 kHz on an ESU output is an alias. Only trust it at ≤200 ns/div. |
| `:MEASure:MATH:ITEM? <any>` | **Returns `0.000e+00` even with live signal** (four real bursts, 11–28 W). The MATH measurement path is dead. |
| `:WAVeform:YINCrement?` | Returns `40000` — garbage. Use `SCALe` ÷ 25 codes/div instead. |

## ❌ No answer at all (7 of 101)

`:ACQuire:DEPMem?` · `:ACQuire:MODE?` · `:TIMebase:OFFSet?` · `:MEASure:MDISplay?` ·
`:WAVeform:PREamble?` · `:WAVeform:MODE?` · `*OPT?`

The first three are **wrong mnemonics on our side**, not firmware gaps — the real ones are
`:ACQuire:POINts`, `:ACQuire:TYPE` and `:TIMebase:POSition`.

## 📌 There is no on-scope V·I on this firmware

`:MATH:OPERator MULTiply` genuinely engages (`:MATH:SCALe?` re-ranges when you set it), but
**the MATH trace is never exported** — with math ON, `PRIVate:WAVeform:DATA:ALL?` still
returns `8000` bytes = 2 ch × 4000, channel-enable field `1100` — and
`:MEASure:MATH:ITEM?` reads zero on live signal. Both routes are closed. Compute `mean(v·i)`
on the PC, where you also keep the three-way cross-check (`Vrms²/R`, `Irms²·R`, `mean(v·i)`)
that catches probe-placement, polarity and coil-scaling errors.

---

## Talking to it from Linux / WSL

The scope is USBTMC (interface class 254, subclass 3). WSL has **no `usbtmc` kernel
module**, so the path is libusb — the same route as Zadig/WinUSB on Windows:

```bash
# from Windows, once:  usbipd bind --busid <id>   (Administrator)
#                      usbipd attach --wsl --busid <id>
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="049f", ATTR{idProduct}=="505e", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/60-hantek.rules      # else the node is root:root
pip install pyvisa pyvisa-py pyusb numpy matplotlib
python esu_test.py --list                           # USB0::1183::20574::CN…::0::INSTR
```

⚠ **Never call `usb.core.Device.reset()` on a usbip-forwarded device.** vhci_hcd cannot do
it — `[Errno 2] Entity not found` — and it drops the scope off the bus entirely, needing an
Administrator `usbipd bind` + `attach` to get back.

⚠ **A killed script can leave the response buffer desynced** — the next `*IDN?` returns the
*previous* query's reply. That is not a firmware fault. Recover with the USBTMC sequence:
`INITIATE_ABORT_BULK_IN` → `CHECK_ABORT_BULK_IN_STATUS` → `INITIATE_CLEAR` →
`CHECK_CLEAR_STATUS` → **`clear_halt` on the Bulk-IN endpoint** (USBTMC 1.00 §4.2.1.6).
Clearing Bulk-OUT instead — the obvious-looking mistake — leaves it stuck forever.

📌 An unrecognised command is **harmless**: it times out, `*IDN?` still answers, and
`:SYSTem:ERRor?` returns `-113`. Only a wrong *recovery* wedges the session.

## Timing (fw 1.0.8) — why the tool is shaped the way it is

| operation | cost |
|---|---|
| any `:MEASure:…:ITEM?` query | **~1 s** |
| `:TRIGger:STATus?` | ~225 ms |
| capture @ 4K / @ 40K | 2.1 s / 5.5 s |
| `autoscale` (V/div only) | **12.8 s** |

`:MEASure` being ~1 s is why `--watch` polls at ~1 Hz, and why the dead VRMS/FREQ/MATH reads
were removed from the burst path. `autoscale` at 12.8 s is why the envelope path captures
first and re-ranges only when clipping is actually detected.
