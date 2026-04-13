@echo off
setlocal enabledelayedexpansion
title PET Bottle Robot - OTA Flash Tool
color 0A

:: ===================== CONFIG =====================
set FQBN=esp32:esp32:esp32:PartitionScheme=min_spiffs
set SKETCH=robotic_arm\robotic_arm.ino
set BIN=build_robotic\robotic_arm.ino.bin
set COPY_TO=robotic_arm\robotic_arm.bin
set ESP_SSID=PetBottle_Robot
set ESP_IP=192.168.4.1
set OTA_URL=http://%ESP_IP%/ota
set TIMEOUT=10

:: ===================== STEP 1: COMPILE =====================
echo.
echo ============================================
echo   STEP 1: Compiling firmware...
echo ============================================
echo.

arduino-cli compile --fqbn %FQBN% --output-dir build_robotic %SKETCH%
if errorlevel 1 (
    echo.
    echo [ERROR] Compilation failed! Fix errors above and try again.
    pause
    exit /b 1
)

:: Copy bin to easy-access location
copy /Y "%BIN%" "%COPY_TO%" >nul 2>&1
echo.
echo [OK] Compiled successfully: %BIN%

:: ===================== STEP 2: REMEMBER HOME WIFI =====================
echo.
echo ============================================
echo   STEP 2: Saving current WiFi...
echo ============================================
echo.

for /f "tokens=2 delims=:" %%a in ('netsh wlan show interfaces ^| findstr /C:"SSID" ^| findstr /V "BSSID"') do (
    set "HOME_SSID=%%a"
    goto :got_ssid
)
:got_ssid
:: Trim leading space
set HOME_SSID=%HOME_SSID:~1%
echo [OK] Home WiFi: %HOME_SSID%

:: ===================== STEP 3: SWITCH TO ESP32 WIFI =====================
echo.
echo ============================================
echo   STEP 3: Connecting to %ESP_SSID%...
echo ============================================
echo.

netsh wlan connect name="%ESP_SSID%" >nul 2>&1

:: Wait for connection (up to 15 seconds)
set CONNECTED=0
for /L %%i in (1,1,15) do (
    if !CONNECTED! == 0 (
        timeout /t 1 /nobreak >nul
        for /f "tokens=2 delims=:" %%a in ('netsh wlan show interfaces ^| findstr /C:"SSID" ^| findstr /V "BSSID"') do (
            echo %%a | findstr /C:"%ESP_SSID%" >nul && set CONNECTED=1
        )
    )
)

if %CONNECTED% == 0 (
    echo [ERROR] Could not connect to %ESP_SSID%.
    echo         Make sure the robot is powered on and in range.
    echo         Reconnecting to %HOME_SSID%...
    netsh wlan connect name="%HOME_SSID%" >nul 2>&1
    pause
    exit /b 1
)
echo [OK] Connected to %ESP_SSID%

:: Wait a moment for IP assignment
timeout /t 2 /nobreak >nul

:: ===================== STEP 4: UPLOAD VIA OTA =====================
echo.
echo ============================================
echo   STEP 4: Uploading firmware via OTA...
echo ============================================
echo.
echo Uploading %BIN% to %OTA_URL% ...
echo (This takes ~20-30 seconds, do NOT power off the robot)
echo.

curl -s -o ota_response.tmp -w "%%{http_code}" --connect-timeout %TIMEOUT% -X POST -F "update=@%BIN%" %OTA_URL% > ota_status.tmp 2>&1
set /p HTTP_STATUS=<ota_status.tmp

if "%HTTP_STATUS%" == "200" (
    echo.
    echo [OK] OTA upload successful! Robot is rebooting...
    echo     Waiting 5 seconds for reboot...
    timeout /t 5 /nobreak >nul
) else (
    echo.
    echo [ERROR] OTA upload failed! HTTP status: %HTTP_STATUS%
    if exist ota_response.tmp type ota_response.tmp
    echo.
)

:: Cleanup temp files
del /q ota_response.tmp ota_status.tmp 2>nul

:: ===================== STEP 5: SWITCH BACK TO HOME WIFI =====================
echo.
echo ============================================
echo   STEP 5: Reconnecting to %HOME_SSID%...
echo ============================================
echo.

netsh wlan connect name="%HOME_SSID%" >nul 2>&1
timeout /t 3 /nobreak >nul
echo [OK] Back on %HOME_SSID%

:: ===================== DONE =====================
echo.
echo ============================================
echo   DONE! Firmware flashed via OTA.
echo ============================================
echo.
echo   To verify: connect phone to %ESP_SSID%
echo   and open http://%ESP_IP%/
echo.
pause
