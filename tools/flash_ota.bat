@echo off
setlocal enabledelayedexpansion
title PET Bottle Robot - OTA Flash Tool
color 0A

:: ===================== CONFIG =====================
set FQBN=esp32:esp32:esp32:PartitionScheme=min_spiffs
set SKETCH=esp32_test\esp32_test.ino
set BIN=build_test\esp32_test.ino.bin
set COPY_TO=esp32_test\esp32_test.bin
set HOTSPOT_SSID=petbottle_hotspot
set ESP_IP=
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

:: ===================== STEP 2: CHECK HOTSPOT CONNECTION =====================
echo.
echo ============================================
echo   STEP 2: Checking hotspot connection...
echo ============================================
echo.
echo Make sure your PC is connected to the %HOTSPOT_SSID% hotspot.
echo ESP32 should be reachable at %ESP_IP%.
echo.

:: Quick ping check
ping -n 1 -w 2000 %ESP_IP% >nul 2>&1
if errorlevel 1 (
    echo [WARNING] Cannot reach ESP32 at %ESP_IP%.
    echo           Make sure hotspot is on and ESP32 is connected.
    echo           Press any key to try OTA anyway, or Ctrl+C to abort.
    pause >nul
)

:: ===================== STEP 3: UPLOAD VIA OTA =====================
echo.
echo ============================================
echo   STEP 3: Uploading firmware via OTA...
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

:: ===================== DONE =====================
echo.
echo ============================================
echo   DONE! Firmware flashed via OTA.
echo ============================================
echo.
echo   To verify: open http://%ESP_IP%/
echo.
pause
