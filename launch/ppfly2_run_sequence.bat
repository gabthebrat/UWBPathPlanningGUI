@REM Start UWBViz/main.py to mark positions of Victims, Danger Zones and Pillars
python -m UWBViz.main
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] UWBViz/main.py encountered an error. Continuing with the next steps...
)

@REM Run PPGUI/main.py to mark waypoints
python PPGUI/main.py
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
start python -m UWBViz.main
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] UWBViz/main.py encountered an error. Continuing with the next steps...
)

@REM Wait 3 seconds before starting the flight routine
timeout /t 3 /nobreak

@REM Run PPFLY2/main.py for the flight routine (add --simulate and -f flags if needed)
@REM python -m PPFLY2.main --sim 1 --f waypoints_samplesmall.json
python -m PPFLY2.main shared_params.params
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] PPFLY/main.py encountered an error.
)

echo [INFO] Exiting script.
exit /b

