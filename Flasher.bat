@echo off
rem Skipper Flipper Flasher (needs Python 3.8+ from python.org with Tk)
cd /d "%~dp0"
set PYTHONUTF8=1
where py >nul 2>nul
if %errorlevel%==0 (
  py -3 "%~dp0flasher.py"
) else (
  python "%~dp0flasher.py"
)
if errorlevel 1 pause
