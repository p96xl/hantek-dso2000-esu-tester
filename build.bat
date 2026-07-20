@echo off
REM Build a single portable, OFFLINE esu_test.exe (run on Windows, needs Python once here).
REM Driver on target PCs = Zadig -> bind scope USBTMC interface to WinUSB. No NI-VISA.

pip install -r requirements.txt libusb-package pyinstaller

REM Grab the bundled libusb-1.0.dll from the libusb-package wheel (offline after pip).
python -c "import libusb_package,glob,os,shutil,sys; f=glob.glob(os.path.join(os.path.dirname(libusb_package.__file__),'**','libusb-1.0.dll'),recursive=True); shutil.copy(f[0],'.') if f else sys.exit('libusb-1.0.dll not found - drop it here manually from libusb.info')"

REM Embed the DLL INTO the onefile exe so only one file travels.
REM --windowed = no background console when techs double-click the GUI.
REM (CLI text modes like --calcheck/--list still run, but print nowhere on the exe; run those from `python esu_test.py`.)
pyinstaller --onefile --windowed --collect-all pyvisa --collect-all pyvisa_py --collect-all usb --add-binary "libusb-1.0.dll;." esu_test.py

echo.
echo Done. Single file is dist\esu_test.exe  -- copy it anywhere.
echo On each target PC once: run Zadig, pick the scope USBTMC interface, Install WinUSB.
echo Then run:  esu_test.exe --list   to confirm the scope is seen.
