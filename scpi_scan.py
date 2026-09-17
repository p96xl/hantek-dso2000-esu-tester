#!/usr/bin/env python3
"""Definitive SCPI capability probe for the DSO2C50 (DSO2000 series), fw 1.0.8.
Answers ONE question: does :MEASure actually work so we can read the scope's own
frequency/Vrms counters instead of computing them? Tries the manual's exact syntax
(with :MEASure:ENABle ON first) plus every abbreviation/variant.

Run (scope in Utility->I/O->USB Device=Computer/USBTMC, WinUSB bound via Zadig):
    python scpi_scan.py

Each line prints:  OK <response> | <empty> | Timeout | Err.  Have the ESU running (or a
probe-comp square wave on CH1) so the measurements have a real signal to chew on.
"""
from esu_test import connect


def main():
    scope = connect()
    scope.timeout = 1500          # short so a dead command fails fast instead of the 15 s default

    def q(cmd):
        try:
            r = scope.query(cmd).strip()
            return f"OK   {r!r}" if r else "OK   <empty>"
        except Exception as e:
            return f"{type(e).__name__}: {str(e)[:50]}"

    def w(cmd):
        try:
            scope.write(cmd); return "sent"
        except Exception as e:
            return f"{type(e).__name__}"

    def err():                    # -113 'undefined header' => firmware doesn't know the command
        try:
            return scope.query(':SYSTem:ERRor?').strip()
        except Exception as e:
            return f"<{type(e).__name__}>"

    def row(cmd):
        print(f"  {cmd:44s} -> {q(cmd)}")

    print("\n=== basics ===")
    for c in ('*IDN?', '*OPC?', ':SYSTem:ERRor?'):
        row(c)

    print("\n=== MEASure subsystem — enable it FIRST (the likely missing step) ===")
    for c in (':MEASure:ENABle ON', ':MEASure:ADISplay ON', ':MEASure:SOURce CHANnel1'):
        print(f"  WRITE {c:38s} -> {w(c):6s}  err: {err()}")
    for c in (':MEASure:ENABle?', ':MEASure:SOURce?', ':MEASure:ADISplay?'):
        row(c)

    ITEMS = ('FREQuency', 'PERiod', 'VRMS', 'VPP', 'VMAX', 'VAVG')

    print("\n  -- form A: :MEASure:CHANnel1:ITEM? <type> --")
    for it in ITEMS:
        row(f':MEASure:CHANnel1:ITEM? {it}')

    print("\n  -- form B: set :ITEM <type> then query :ITEM? --")
    for it in ITEMS:
        print(f"  set {it:12s} ({w(f':MEASure:CHANnel1:ITEM {it}')})  err:{err():>6}  ", end='')
        row(':MEASure:CHANnel1:ITEM?')

    print("\n  -- form C: abbreviations / alt argument order --")
    for c in (':MEAS:CHAN1:ITEM? FREQ', ':MEASure:ITEM? FREQuency,CHANnel1',
              ':MEAS:FREQ? CHANnel1', ':MEASure:FREQuency? CHANnel1'):
        row(c)

    print("\n=== other maybe-useful queries ===")
    for c in (':ACQuire:POINts?', ':ACQuire:SRATe?', ':TRIGger:STATus?',
              ':WAVeform:XINCrement?', ':FUNCtion:WINDow?'):
        row(c)

    scope.close()
    print("\n--- verdict ---")
    print("Form A/B/C returns NUMBERS  -> YES, we can read the scope's counters (drop the FFT).")
    print("All Timeout / -113 undefined -> confirmed DEAD; keep computing from the trace.")


if __name__ == '__main__':
    main()
