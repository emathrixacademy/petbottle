#!/usr/bin/env python3
"""Generate wiring diagram PDF for PET Bottle robot motor control."""

from fpdf import FPDF

pdf = FPDF(orientation="P", unit="mm", format="A4")
pdf.set_auto_page_break(auto=True, margin=15)
pdf.add_page()

# Title
pdf.set_font("Helvetica", "B", 20)
pdf.cell(0, 12, "PET Bottle Robot - Motor Wiring Diagram", ln=True, align="C")
pdf.ln(4)

pdf.set_font("Helvetica", "", 11)
pdf.cell(0, 7, "Motor: MY1016Z  |  24V DC  |  330 RPM  |  13A  |  ~200W", ln=True, align="C")
pdf.cell(0, 7, "Driver: BTS7960 43A (x2)  |  Controller: ESP32", ln=True, align="C")
pdf.ln(6)

# Wiring diagram (monospace)
pdf.set_font("Courier", "", 8)
diagram = """    24V Battery (e.g., 2x 12V SLA in series)
     (+) ----------+------------------+
     (-) -----+----+-------------+    |
              |    |             |    |
         +----+----+----+  +----+----+----+
         |  BTS7960 #1  |  |  BTS7960 #2  |
         |  (LEFT)      |  |  (RIGHT)     |
         |              |  |              |
         | RPWM  LPWM  |  | RPWM  LPWM  |
         |  |      |    |  |  |      |    |
         | M+    M-    |  | M+    M-    |
         +--+------+----+  +--+------+----+
            |      |          |      |
        +---+------+---+  +--+------+---+
        |  LEFT MOTOR  |  | RIGHT MOTOR |
        |  MY1016Z     |  |  MY1016Z    |
        |  24V / 13A   |  |  24V / 13A  |
        +--------------+  +-------------+


         ESP32 Connections:
         +----------------+
         |     ESP32      |
         |                |
         |  GPIO 25 ----------> BTS7960 #1 RPWM (Left Fwd)
         |  GPIO 26 ----------> BTS7960 #1 LPWM (Left Rev)
         |  GPIO 32 ----------> BTS7960 #1 R_EN + L_EN
         |                |
         |  GPIO 27 ----------> BTS7960 #2 RPWM (Right Fwd)
         |  GPIO 14 ----------> BTS7960 #2 LPWM (Right Rev)
         |  GPIO 33 ----------> BTS7960 #2 R_EN + L_EN
         |                |
         |  GND -----------> Common GND (all drivers + battery)
         |  3.3V ----------> BTS7960 VCC (both)
         +----------------+"""

for line in diagram.split("\n"):
    pdf.cell(0, 4, line, ln=True)

pdf.ln(6)

# BTS7960 #1 wiring table
pdf.set_font("Helvetica", "B", 13)
pdf.cell(0, 9, "BTS7960 #1 - Left Motor Wiring", ln=True)
pdf.ln(2)

pdf.set_font("Helvetica", "B", 10)
col_w = [45, 45, 80]
headers = ["BTS7960 Pin", "Connect To", "Purpose"]
for i, h in enumerate(headers):
    pdf.cell(col_w[i], 8, h, border=1, align="C")
pdf.ln()

pdf.set_font("Helvetica", "", 10)
rows1 = [
    ("RPWM", "ESP32 GPIO 25", "Forward PWM"),
    ("LPWM", "ESP32 GPIO 26", "Reverse PWM"),
    ("R_EN + L_EN", "ESP32 GPIO 32", "Enable (or tie HIGH)"),
    ("VCC", "ESP32 3.3V", "Logic power"),
    ("GND", "Common GND", "ESP32 + Battery GND"),
    ("B+", "Battery 24V (+)", "Motor power supply"),
    ("B-", "Battery 24V (-)", "Motor power supply"),
    ("M+", "Left Motor wire 1", "Motor output"),
    ("M-", "Left Motor wire 2", "Motor output"),
]
for row in rows1:
    for i, val in enumerate(row):
        pdf.cell(col_w[i], 7, val, border=1)
    pdf.ln()

pdf.ln(6)

# BTS7960 #2 wiring table
pdf.set_font("Helvetica", "B", 13)
pdf.cell(0, 9, "BTS7960 #2 - Right Motor Wiring", ln=True)
pdf.ln(2)

pdf.set_font("Helvetica", "B", 10)
for i, h in enumerate(headers):
    pdf.cell(col_w[i], 8, h, border=1, align="C")
pdf.ln()

pdf.set_font("Helvetica", "", 10)
rows2 = [
    ("RPWM", "ESP32 GPIO 27", "Forward PWM"),
    ("LPWM", "ESP32 GPIO 14", "Reverse PWM"),
    ("R_EN + L_EN", "ESP32 GPIO 33", "Enable (or tie HIGH)"),
    ("VCC", "ESP32 3.3V", "Logic power"),
    ("GND", "Common GND", "ESP32 + Battery GND"),
    ("B+", "Battery 24V (+)", "Motor power supply"),
    ("B-", "Battery 24V (-)", "Motor power supply"),
    ("M+", "Right Motor wire 1", "Motor output"),
    ("M-", "Right Motor wire 2", "Motor output"),
]
for row in rows2:
    for i, val in enumerate(row):
        pdf.cell(col_w[i], 7, val, border=1)
    pdf.ln()

pdf.ln(6)

# Motor direction table
pdf.set_font("Helvetica", "B", 13)
pdf.cell(0, 9, "Motor Direction Logic", ln=True)
pdf.ln(2)

pdf.set_font("Helvetica", "B", 10)
dir_headers = ["RPWM", "LPWM", "Motor Action"]
dir_w = [40, 40, 90]
for i, h in enumerate(dir_headers):
    pdf.cell(dir_w[i], 8, h, border=1, align="C")
pdf.ln()

pdf.set_font("Helvetica", "", 10)
dir_rows = [
    ("PWM (0-255)", "LOW (0)", "Forward (speed = PWM duty)"),
    ("LOW (0)", "PWM (0-255)", "Reverse (speed = PWM duty)"),
    ("LOW (0)", "LOW (0)", "Stop (coast)"),
]
for row in dir_rows:
    for i, val in enumerate(row):
        pdf.cell(dir_w[i], 7, val, border=1)
    pdf.ln()

pdf.ln(6)

# Important notes
pdf.set_font("Helvetica", "B", 13)
pdf.cell(0, 9, "Important Notes", ln=True)
pdf.ln(2)

pdf.set_font("Helvetica", "", 10)
notes = [
    "- Battery: 24V, minimum 20Ah recommended (2x 12V SLA in series or 24V 6S LiPo)",
    "- Wiring gauge: Use 12-14 AWG wire for motor power lines (13A per motor)",
    "- Fuse: Add a 30A inline fuse between battery and drivers",
    "- Common GND is CRITICAL: ESP32, both BTS7960 drivers, and battery must share ground",
    "- Remove ENA/ENB jumpers if present on BTS7960 to enable PWM speed control",
    "- PWM frequency: 20kHz recommended for quiet motor operation",
]
for note in notes:
    pdf.cell(0, 6, note, ln=True)

# Save
out = "/home/user/Desktop/Projects/petbottle/wiring_diagram.pdf"
pdf.output(out)
print(f"PDF saved to: {out}")
