@echo off
REM ============================================================
REM  Build robotic_arm.ino and copy the .bin next to the sketch
REM  Output: robotic_arm\robotic_arm.bin  (always overwritten)
REM ============================================================

cd /d "%~dp0"

echo.
echo === Compiling robotic_arm.ino ===
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --output-dir build_robotic robotic_arm\robotic_arm.ino
if errorlevel 1 (
    echo.
    echo *** BUILD FAILED ***
    pause
    exit /b 1
)

copy /Y build_robotic\robotic_arm.ino.bin robotic_arm\robotic_arm.bin >nul

echo.
echo === BUILD OK ===
echo Binary: %cd%\robotic_arm\robotic_arm.bin
echo.
echo Next steps:
echo   1. Connect to WiFi:  PetBottle_Robot  (password: petbottle123)
echo   2. Open browser:     http://192.168.4.1/ota
echo   3. Choose file:      robotic_arm\robotic_arm.bin
echo   4. Click Upload ^& Flash
echo.
pause
