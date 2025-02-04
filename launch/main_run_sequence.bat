@REM Start UWBViz/main.py to mark positions of Victims, Danger Zones and Pillars
python3 UWBViz/main.py
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] UWBViz/main.py encountered an error. Continuing with the next steps...
)

@REM Run PPGUI/main.py to mark waypoints
python3 PPGUI/main.py
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] PPGUI/main.py encountered an error. Continuing with the next steps...
)

@REM Run nlink_unpack_COMx_udp and check if it runs successfully
nlink_unpack_COMx_udp
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] nlink_unpack_COMx_udp failed. Continuing with the next steps...
) else (
    echo [INFO] nlink_unpack_COMx_udp is running in the background.
)

@REM Prompt the user to confirm WiFi connection to Tello
echo [INFO] Ensure WiFi is connected to Tello for flight.
choice /c EY /n /m "Press Y to continue or E to exit."
if %ERRORLEVEL% EQU 1 (
    echo [INFO] Exiting script.
    exit /b
)


@REM Start UWBViz/main.py in a new process (i.e. open a new terminal)
start python3 UWBViz/main.py
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] UWBViz/main.py encountered an error. Continuing with the next steps...
)

@REM Wait 3 seconds before starting the flight routine
timeout /t 3 /nobreak

@REM Run PPFLY/main.py for the flight routine (add --simulate flag if needed)
python3 PPFLY/main.py --simulate
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] PPFLY/main.py encountered an error.
)

echo [INFO] Exiting script.
exit /b

