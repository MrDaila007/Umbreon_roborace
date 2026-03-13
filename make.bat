@echo off
setlocal

set PYTHON=python
set PIP=pip
set SIM_DIR=simulation
set DASH_DIR=dashboard
set PORT=8080
set SIM_PORT=8023

if "%1"=="" goto help
if "%1"=="help" goto help
if "%1"=="install" goto install
if "%1"=="dashboard" goto web
if "%1"=="web" goto web
if "%1"=="dashboard-tk" goto dashboard_tk
if "%1"=="sim" goto sim
if "%1"=="sim-fast" goto sim_fast
if "%1"=="bridge" goto bridge
if "%1"=="sim-bridge" goto bridge
if "%1"=="demo" goto demo
if "%1"=="clean" goto clean
echo Unknown target: %1
goto help

:install
echo Installing dependencies...
%PIP% install -r %DASH_DIR%\requirements.txt
goto :eof

:web
echo Starting web dashboard on http://localhost:%PORT%
cd %DASH_DIR% && %PYTHON% server.py %PORT%
goto :eof

:dashboard_tk
echo Starting tkinter dashboard...
cd %DASH_DIR% && %PYTHON% app.py
goto :eof

:sim
echo Running live simulation...
cd %SIM_DIR% && %PYTHON% sim.py
goto :eof

:sim_fast
echo Running pre-computed simulation...
cd %SIM_DIR% && %PYTHON% sim.py --fast
goto :eof

:bridge
echo Running simulation with TCP bridge on port %SIM_PORT%...
cd %SIM_DIR% && %PYTHON% sim.py --bridge
goto :eof

:demo
echo Starting simulator bridge on port %SIM_PORT%...
start "Umbreon Sim Bridge" cmd /c "cd %SIM_DIR% && %PYTHON% sim.py --bridge"
timeout /t 2 /nobreak >nul
echo Starting web dashboard on http://localhost:%PORT%
echo Connect to localhost:%SIM_PORT% in the dashboard
cd %DASH_DIR% && %PYTHON% server.py %PORT%
goto :eof

:clean
echo Cleaning Python caches...
for /d /r . %%d in (__pycache__) do @if exist "%%d" rd /s /q "%%d"
del /s /q *.pyc 2>nul
echo Done.
goto :eof

:help
echo.
echo   Umbreon Roborace
echo   ─────────────────────────────────────────
echo   make install       Install Python dependencies
echo   make web           Start web dashboard (localhost:8080)
echo   make dashboard-tk  Start tkinter desktop dashboard
echo   make sim           Run live simulation
echo   make sim-fast      Run pre-computed simulation
echo   make bridge        Run sim with TCP bridge for dashboard
echo   make demo          Start sim bridge + web dashboard together
echo   make clean         Remove Python caches
echo.
goto :eof
