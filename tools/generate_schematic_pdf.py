from fpdf import FPDF

class SchematicPDF(FPDF):
    def header(self):
        self.set_font("Helvetica", "B", 16)
        self.cell(0, 10, "PET Bottle Robot - Complete Wiring Schematic", align="C", new_x="LMARGIN", new_y="NEXT")
        self.set_font("Helvetica", "", 10)
        self.cell(0, 6, "ESP32 | Raspberry Pi | Hailo-8 | 3x BTS7960B | L298N | 2x MG996R | 2x HC-SR04 | LCD I2C | 24V", align="C", new_x="LMARGIN", new_y="NEXT")
        self.ln(4)

    def footer(self):
        self.set_y(-15)
        self.set_font("Helvetica", "I", 8)
        self.cell(0, 10, f"Page {self.page_no()}/{{nb}}", align="C")

    def section_title(self, title):
        self.set_font("Helvetica", "B", 13)
        self.set_fill_color(40, 40, 40)
        self.set_text_color(255, 255, 255)
        self.cell(0, 9, f"  {title}", fill=True, new_x="LMARGIN", new_y="NEXT")
        self.set_text_color(0, 0, 0)
        self.ln(3)

    def sub_title(self, title):
        self.set_font("Helvetica", "B", 11)
        self.set_text_color(30, 30, 120)
        self.cell(0, 7, title, new_x="LMARGIN", new_y="NEXT")
        self.set_text_color(0, 0, 0)
        self.ln(1)

    def body_text(self, text):
        self.set_font("Helvetica", "", 10)
        self.multi_cell(0, 5.5, text)
        self.ln(1)

    def mono_block(self, text):
        self.set_font("Courier", "", 8.5)
        for line in text.strip().split("\n"):
            self.cell(0, 4.2, line, new_x="LMARGIN", new_y="NEXT")
        self.ln(2)

    def table_header(self, cols, widths):
        self.set_font("Helvetica", "B", 10)
        self.set_fill_color(220, 220, 220)
        for i, col in enumerate(cols):
            self.cell(widths[i], 7, col, border=1, fill=True)
        self.ln()

    def table_row(self, cols, widths, mono_first=True):
        if mono_first:
            self.set_font("Courier", "", 10)
            self.cell(widths[0], 6.5, cols[0], border=1)
            self.set_font("Helvetica", "", 10)
            for i in range(1, len(cols)):
                self.cell(widths[i], 6.5, cols[i], border=1)
        else:
            self.set_font("Helvetica", "", 10)
            for i, col in enumerate(cols):
                self.cell(widths[i], 6.5, col, border=1)
        self.ln()


pdf = SchematicPDF()
pdf.alias_nb_pages()
pdf.set_auto_page_break(auto=True, margin=20)
pdf.add_page()

# ============================================================
# PAGE 1: System Architecture
# ============================================================
pdf.section_title("System Architecture Overview")

pdf.body_text(
    "The PET Bottle Collector Robot uses two processors:\n"
    "1. Raspberry Pi 5 + Hailo-8 NPU: Camera vision (YOLOv5/v7/v8) + Navigation brain\n"
    "2. ESP32: Motor control, ultrasonic sensors, WiFi AP, LCD display\n"
    "\n"
    "Communication: Raspberry Pi connects to ESP32 WiFi AP (192.168.4.1) and sends\n"
    "HTTP commands. ESP32 provides sensor data via GET /sensor endpoint."
)

pdf.sub_title("Communication Diagram")
pdf.mono_block("""
  Raspberry Pi 5                          ESP32 (WiFi AP)
  +--------------------+      WiFi       +------------------------+
  | CSI Camera         |                 | 192.168.4.1            |
  | Hailo-8 NPU       |                 |                        |
  | YOLOv5/v7/v8      | GET /sensor     | Ultrasonic L/R (cm)    |
  | navigator.py  <----+<----- JSON ----+--> HC-SR04 x2           |
  |                    |                 |                        |
  | Bottle detected?   | GET /cmd?c=F80  | Drive wheels (BTS7960) |
  | Obstacle?     -----+---- command --->| Arm lift (BTS7960)     |
  | Pickup!       -----+---- cmd?c=P --->| Servos (MG996R x2)    |
  |                    |                 | Stepper base (L298N)   |
  +--------------------+                 | LCD 16x2 + Buzzer     |
                                         +------------------------+
""")

pdf.sub_title("Robot State Machine (navigator.py)")
pdf.mono_block("""
     +--------+    bottle     +------------+    centered    +----------+
     | ROAMING|  detected    | APPROACHING|   & close     | ALIGNING |
     | (S-curve+----------->|  (drive to) +-------------->| (fine    |
     |  explore)|            +------+-----+               |  adjust) |
     +----+----+                    |                      +----+-----+
          |                         | lost bottle               |
          |  obstacle               | (15 frames)               | aligned
          |  detected               v                           v
     +----+----+             +------+-----+              +-----+------+
     | AVOIDING|             |   ROAMING  |              | PICKING UP |
     | (back up|             |            |              | (ESP32 P   |
     |  & turn)|             +------------+              |  command)  |
     +---------+                                         +-----+------+
                                                               |
                                                               | done
                                                               v
                                                         +-----+------+
                                                         |   ROAMING  |
                                                         +------------+
""")

# ============================================================
# PAGE 2: GPIO Pin Assignment
# ============================================================
pdf.add_page()
pdf.section_title("ESP32 GPIO Pin Assignment (21 pins used)")

w = [30, 75, 55, 30]
pdf.table_header(["GPIO", "Component", "Function", "Dir"], w)
pins = [
    ("GPIO4",  "BTS7960B #1", "Left Wheel RPWM", "OUT"),
    ("GPIO5",  "BTS7960B #1", "Left Wheel LPWM", "OUT"),
    ("GPIO26", "BTS7960B #1", "Left Wheel EN", "OUT"),
    ("GPIO2",  "BTS7960B #2", "Right Wheel RPWM", "OUT"),
    ("GPIO25", "BTS7960B #2", "Right Wheel LPWM", "OUT"),
    ("GPIO19", "BTS7960B #2", "Right Wheel EN", "OUT"),
    ("GPIO16", "BTS7960B #3", "Arm Lift RPWM", "OUT"),
    ("GPIO17", "BTS7960B #3", "Arm Lift LPWM", "OUT"),
    ("GPIO27", "BTS7960B #3", "Arm Lift EN", "OUT"),
    ("GPIO32", "L298N", "Stepper IN1", "OUT"),
    ("GPIO33", "L298N", "Stepper IN2", "OUT"),
    ("GPIO22", "L298N", "Stepper IN3", "OUT"),
    ("GPIO21", "L298N", "Stepper IN4", "OUT"),
    ("GPIO13", "MG996R Servo L", "Left Servo Signal", "OUT"),
    ("GPIO18", "MG996R Servo R", "Right Servo Signal", "OUT"),
    ("GPIO14", "LCD I2C", "SDA (data)", "I/O"),
    ("GPIO15", "LCD I2C", "SCL (clock)", "OUT"),
    ("GPIO23", "Buzzer", "Alert Signal", "OUT"),
    ("GPIO12", "HC-SR04 x2", "Shared TRIG", "OUT"),
    ("GPIO34", "HC-SR04 Left", "Left ECHO", "IN"),
    ("GPIO35", "HC-SR04 Right", "Right ECHO", "IN"),
]
for p in pins:
    pdf.table_row(list(p), w)

pdf.ln(3)
pdf.set_font("Helvetica", "I", 9)
pdf.cell(0, 6, "Used: 21 pins  |  Free: GPIO 0, 36, 39", new_x="LMARGIN", new_y="NEXT")
pdf.cell(0, 6, "WiFi AP: PetBottle_Robot (192.168.4.1)  |  USB Serial: 115200 baud", new_x="LMARGIN", new_y="NEXT")
pdf.cell(0, 6, "GPIO34, GPIO35 = input-only pins (perfect for ultrasonic ECHO)", new_x="LMARGIN", new_y="NEXT")

# ============================================================
# PAGE 3: Ultrasonic Sensors (NEW)
# ============================================================
pdf.add_page()
pdf.section_title("1. Ultrasonic Sensors - HC-SR04 x2 (Obstacle Avoidance)")

pdf.sub_title("About HC-SR04")
pdf.body_text(
    "- Measures distance via ultrasonic pulse echo (40kHz)\n"
    "- Range: 2cm to 400cm, accuracy +/- 3mm\n"
    "- Operating voltage: 5V\n"
    "- Trigger pulse: 10us HIGH on TRIG pin\n"
    "- ECHO pin returns HIGH pulse proportional to distance\n"
    "- Distance (cm) = pulse duration (us) x 0.034 / 2"
)

pdf.sub_title("Wiring - Left Ultrasonic")
w_us = [55, 50, 85]
pdf.table_header(["HC-SR04 Pin", "Connect To", "Notes"], w_us)
rows_usl = [
    ("VCC", "5V (DC-DC out)", "Must be 5V, not 3.3V"),
    ("TRIG", "GPIO 12", "Shared with right sensor"),
    ("ECHO", "GPIO 34", "Input-only pin (safe for 5V*)"),
    ("GND", "Common GND", "Shared ground"),
]
for r in rows_usl:
    pdf.table_row(list(r), w_us, mono_first=True)

pdf.ln(2)
pdf.sub_title("Wiring - Right Ultrasonic")
pdf.table_header(["HC-SR04 Pin", "Connect To", "Notes"], w_us)
rows_usr = [
    ("VCC", "5V (DC-DC out)", "Must be 5V, not 3.3V"),
    ("TRIG", "GPIO 12", "Shared with left sensor"),
    ("ECHO", "GPIO 35", "Input-only pin (safe for 5V*)"),
    ("GND", "Common GND", "Shared ground"),
]
for r in rows_usr:
    pdf.table_row(list(r), w_us, mono_first=True)

pdf.ln(2)
pdf.body_text(
    "* GPIO 34/35 are input-only pins with no internal pull-up.\n"
    "  HC-SR04 ECHO outputs 5V. For safety, add a voltage divider:\n"
    "  ECHO --> [1K ohm] --> GPIO34/35 --> [2K ohm] --> GND\n"
    "  This divides 5V to ~3.3V. Many ESP32 boards tolerate 5V on\n"
    "  input-only pins, but the divider is recommended."
)

pdf.sub_title("Mounting Position")
pdf.mono_block("""
             FRONT OF ROBOT
        +---------------------+
        |                     |
   [US-L]    [CSI Camera]    [US-R]
    <--       |  lens  |       -->
    30deg     +--------+     30deg
        |                     |
        |   [Arm / Servos]    |
        |                     |
   +----+                     +----+
   | L  |                     | R  |  Wheels
   |Whl |                     |Whl |
   +----+                     +----+
             REAR OF ROBOT

  Mount sensors at front corners
  Angle outward ~30 degrees for wide coverage
  Height: same level as chassis (~10cm from ground)
""")

pdf.sub_title("Obstacle Avoidance Thresholds")
w_th = [50, 50, 90]
pdf.table_header(["Distance", "Action", "Details"], w_th)
thresh_rows = [
    ("< 15 cm", "EMERGENCY STOP", "Full stop, back up, turn away"),
    ("15-40 cm", "SLOW DOWN", "Reduce speed to 50/255"),
    ("40-60 cm", "STEER AWAY", "Turn away from closer side"),
    ("> 60 cm", "CLEAR", "Full roaming speed (80/255)"),
]
for r in thresh_rows:
    pdf.table_row(list(r), w_th, mono_first=False)

pdf.ln(2)
pdf.sub_title("ESP32 /sensor API Endpoint")
pdf.body_text(
    "GET http://192.168.4.1/sensor\n\n"
    "Response (JSON):\n"
    '  {"left": 45, "right": 120,\n'
    '   "wheels": {"left": 80, "right": 80},\n'
    '   "base": 0, "lift": 0, "motors": 1}\n\n'
    "Polled by Raspberry Pi at 10 Hz for real-time obstacle data.\n"
    "Fused with camera person/object detection for 360-degree awareness."
)

# ============================================================
# PAGE 4: BTS7960B Drive Wheels
# ============================================================
pdf.add_page()
pdf.section_title("2. Drive Wheels - BTS7960B (HW-039) x2")

pdf.sub_title("About the BTS7960B")
pdf.body_text(
    "- High-power 43A H-bridge motor driver\n"
    "- Operating voltage: 5.5V to 27V (supports 24V directly)\n"
    "- PWM frequency: up to 25 kHz\n"
    "- Built-in thermal shutdown and overcurrent protection\n"
    "- Each board: RPWM, LPWM, R_EN, L_EN, VCC, GND, B+, B-, M+, M-"
)

pdf.sub_title("BTS7960B #1 - Left Drive Wheel (24V Motor)")
w2 = [55, 50, 85]
pdf.table_header(["BTS7960B Pin", "Connect To", "Notes"], w2)
rows = [
    ("RPWM", "GPIO 4", "Forward PWM (0-255)"),
    ("LPWM", "GPIO 5", "Reverse PWM (0-255)"),
    ("R_EN", "GPIO 26", "Enable (tied with L_EN)"),
    ("L_EN", "GPIO 26", "Enable (tied with R_EN)"),
    ("VCC", "3.3V (ESP32)", "Logic power"),
    ("GND", "Common GND", "Shared ground"),
    ("B+", "24V Battery +", "Via 10A fuse"),
    ("B-", "24V Battery -", "Motor power ground"),
    ("M+", "Left Motor +", "To left wheel motor"),
    ("M-", "Left Motor -", "To left wheel motor"),
]
for r in rows:
    pdf.table_row(list(r), w2, mono_first=True)

pdf.ln(3)
pdf.sub_title("BTS7960B #2 - Right Drive Wheel (24V Motor)")
pdf.table_header(["BTS7960B Pin", "Connect To", "Notes"], w2)
rows2 = [
    ("RPWM", "GPIO 2", "Forward PWM (boot-safe pin)"),
    ("LPWM", "GPIO 25", "Reverse PWM (0-255)"),
    ("R_EN", "GPIO 19", "Enable (tied with L_EN)"),
    ("L_EN", "GPIO 19", "Enable (tied with R_EN)"),
    ("VCC", "3.3V (ESP32)", "Logic power"),
    ("GND", "Common GND", "Shared ground"),
    ("B+", "24V Battery +", "Via 10A fuse"),
    ("B-", "24V Battery -", "Motor power ground"),
    ("M+", "Right Motor +", "To right wheel motor"),
    ("M-", "Right Motor -", "To right wheel motor"),
]
for r in rows2:
    pdf.table_row(list(r), w2, mono_first=True)

pdf.ln(3)
pdf.sub_title("BTS7960B Control Logic")
w3 = [35, 35, 35, 85]
pdf.table_header(["RPWM", "LPWM", "EN", "Action"], w3)
pdf.table_row(["PWM", "0", "HIGH", "Forward (variable speed)"], w3, mono_first=False)
pdf.table_row(["0", "PWM", "HIGH", "Reverse (variable speed)"], w3, mono_first=False)
pdf.table_row(["0", "0", "HIGH", "Coast stop"], w3, mono_first=False)
pdf.table_row(["X", "X", "LOW", "Disabled (emergency stop)"], w3, mono_first=False)

# ============================================================
# PAGE 5: BTS7960B Arm Lift
# ============================================================
pdf.add_page()
pdf.section_title("3. Arm Lift - BTS7960B #3 (58SW31ZY Motor)")

pdf.sub_title("Motor: 58SW31ZY DC Geared Motor")
pdf.body_text(
    "- Voltage: 12-24V DC (runs on 24V from battery)\n"
    "- Speed: 16 RPM (geared down for torque)\n"
    "- Used for: Robotic arm vertical lift mechanism"
)

pdf.sub_title("BTS7960B #3 Wiring")
pdf.table_header(["BTS7960B Pin", "Connect To", "Notes"], w2)
rows3 = [
    ("RPWM", "GPIO 16", "Lift UP PWM (0-255)"),
    ("LPWM", "GPIO 17", "Lift DOWN PWM (0-255)"),
    ("R_EN", "GPIO 27", "Enable (tied with L_EN)"),
    ("L_EN", "GPIO 27", "Enable (tied with R_EN)"),
    ("VCC", "3.3V (ESP32)", "Logic power"),
    ("GND", "Common GND", "Shared ground"),
    ("B+", "24V Battery +", "Via 10A fuse"),
    ("B-", "24V Battery -", "Motor power ground"),
    ("M+", "58SW31ZY +", "To arm lift motor"),
    ("M-", "58SW31ZY -", "To arm lift motor"),
]
for r in rows3:
    pdf.table_row(list(r), w2, mono_first=True)

# ============================================================
# PAGE 6: L298N Stepper
# ============================================================
pdf.add_page()
pdf.section_title("4. Base Platform - NEMA 17 Stepper via L298N")

pdf.sub_title("Motor: NEMA 17 (200 steps/rev, 1.8 deg/step)")
pdf.body_text(
    "- Used for: Circular turning platform (base rotation)\n"
    "- Max rotation: +/- 270 degrees\n"
    "- Driven in FULL4WIRE mode via AccelStepper library\n"
    "- Coils released after each move to save power"
)

pdf.sub_title("L298N Wiring")
w4 = [50, 50, 90]
pdf.table_header(["L298N Pin", "Connect To", "Notes"], w4)
rows4 = [
    ("+12V", "24V Battery", "Motor power (L298N supports up to 35V)"),
    ("GND", "Common GND", "Shared ground"),
    ("IN1", "GPIO 32", "Stepper coil A+"),
    ("IN2", "GPIO 33", "Stepper coil A-"),
    ("IN3", "GPIO 22", "Stepper coil B+"),
    ("IN4", "GPIO 21", "Stepper coil B-"),
    ("ENA", "Jumper ON", "Full current to coil A"),
    ("ENB", "Jumper ON", "Full current to coil B"),
    ("OUT1/OUT2", "NEMA17 Coil A", "Motor coil pair 1"),
    ("OUT3/OUT4", "NEMA17 Coil B", "Motor coil pair 2"),
]
for r in rows4:
    pdf.table_row(list(r), w4, mono_first=True)

# ============================================================
# PAGE 7: Servos + LCD + Buzzer
# ============================================================
pdf.add_page()
pdf.section_title("5. Servo Motors - MG996R x2 (Left & Right Arm)")

pdf.sub_title("Left Servo (GPIO 13)")
w5 = [55, 50, 85]
pdf.table_header(["Servo Wire", "Connect To", "Notes"], w5)
rows5a = [
    ("Orange/Signal", "GPIO 13", "PWM control (50Hz, LEDC ch6)"),
    ("Red/VCC", "5V (DC-DC out)", "Dedicated 5V 2A+ source"),
    ("Brown/GND", "Common GND", "Shared ground"),
]
for r in rows5a:
    pdf.table_row(list(r), w5, mono_first=True)

pdf.ln(2)
pdf.sub_title("Right Servo (GPIO 18)")
pdf.table_header(["Servo Wire", "Connect To", "Notes"], w5)
rows5b = [
    ("Orange/Signal", "GPIO 18", "PWM control (50Hz, LEDC ch7)"),
    ("Red/VCC", "5V (DC-DC out)", "Dedicated 5V 2A+ source"),
    ("Brown/GND", "Common GND", "Shared ground"),
]
for r in rows5b:
    pdf.table_row(list(r), w5, mono_first=True)

pdf.body_text(
    "- Open = 0 deg, Close = 90 deg\n"
    "- O = both open, C = both close\n"
    "- WARNING: Each MG996R draws up to 2.5A stall.\n"
    "  Use a 5V DC-DC converter rated 5A+ for both servos."
)

pdf.ln(2)
pdf.section_title("6. LCD I2C Display (16x2)")
w5c = [55, 50, 85]
pdf.table_header(["LCD Pin", "Connect To", "Notes"], w5c)
rows_lcd = [
    ("SDA", "GPIO 14", "I2C data line"),
    ("SCL", "GPIO 15", "I2C clock line"),
    ("VCC", "5V (DC-DC out)", "Display power"),
    ("GND", "Common GND", "Shared ground"),
]
for r in rows_lcd:
    pdf.table_row(list(r), w5c, mono_first=True)

pdf.body_text(
    "- I2C address: 0x27 (or 0x3F)\n"
    "- Shows: wheel speed, base angle, lift status, IP address\n"
    "- Updates every 500ms"
)

pdf.ln(2)
pdf.section_title("7. Buzzer")
pdf.body_text(
    "GPIO 23  -->  Buzzer (+)\n"
    "GND      -->  Buzzer (-)\n"
    "\n"
    "Active buzzer for startup beep, sequence feedback, emergency alerts."
)

# ============================================================
# PAGE 8: Power Distribution
# ============================================================
pdf.add_page()
pdf.section_title("8. Power Distribution")

pdf.sub_title("Complete Power Wiring Diagram")
pdf.mono_block("""
  [24V 30000mAh ALEAIVY Battery]
     |         |         |         |         |
  [FUSE]    [FUSE]    [FUSE]    [L298N]   [DC-DC 24V->5V 5A]
   10A       10A       10A      Stepper       |
     |         |         |                    |
  [BTS#1]   [BTS#2]   [BTS#3]                +----> ESP32 VIN (5V)
  L Wheel   R Wheel   Arm Lift               |        |
                                              |        +-> 3.3V -> BTS VCC (x3)
                                              |
                                              +----> MG996R Left Servo VCC
                                              +----> MG996R Right Servo VCC
                                              +----> LCD I2C VCC
                                              +----> Buzzer +
                                              +----> HC-SR04 Left VCC
                                              +----> HC-SR04 Right VCC

  [ALL GNDs tied to battery negative]
  Battery GND -- DC-DC GND -- ESP32 GND -- BTS GND (x3)
              -- L298N GND -- Servo GND (x2) -- LCD GND
              -- Buzzer GND -- HC-SR04 GND (x2)
""")

pdf.sub_title("Power Budget")
w_pwr = [70, 40, 80]
pdf.table_header(["Component", "Voltage", "Max Current"], w_pwr)
pwr_rows = [
    ("Left Drive Motor", "24V", "10A (stall)"),
    ("Right Drive Motor", "24V", "10A (stall)"),
    ("Arm Lift Motor", "24V", "5A (stall)"),
    ("NEMA 17 Stepper", "24V", "1.5A"),
    ("MG996R Servo x2", "5V", "5A total (stall)"),
    ("ESP32", "5V", "0.5A"),
    ("LCD I2C", "5V", "0.1A"),
    ("HC-SR04 x2", "5V", "0.1A"),
    ("Buzzer", "5V", "0.05A"),
]
for r in pwr_rows:
    pdf.table_row(list(r), w_pwr, mono_first=False)

# ============================================================
# PAGE 9: Raspberry Pi + Hailo-8
# ============================================================
pdf.add_page()
pdf.section_title("9. Raspberry Pi 5 + Hailo-8 NPU")

pdf.sub_title("Raspberry Pi Connections")
pdf.body_text(
    "The Raspberry Pi runs the vision system (YOLOv5/v7/v8) and navigator.\n"
    "It connects to the ESP32 via WiFi - no physical wiring between them."
)

w_pi = [55, 60, 75]
pdf.table_header(["Pi Port", "Connect To", "Notes"], w_pi)
pi_rows = [
    ("CSI Camera", "Pi Camera V2/V3", "Ribbon cable to CSI connector"),
    ("M.2 slot", "Hailo-8 NPU", "AI accelerator for YOLO inference"),
    ("USB-C", "5V 5A PSU", "Official Pi 5 power supply"),
    ("WiFi", "ESP32 AP", "SSID: PetBottle_Robot"),
]
for r in pi_rows:
    pdf.table_row(list(r), w_pi, mono_first=True)

pdf.ln(3)
pdf.sub_title("Software Stack on Raspberry Pi")
pdf.body_text(
    "camera.py     - YOLO model stitching (v5/v7/v8), detection & postprocessing\n"
    "navigator.py  - Autonomous brain: roaming, obstacle avoidance, bottle pickup\n"
    "server.py     - OTA update server for remote code deployment\n"
    "\n"
    "YOLO Models (Hailo HEF format):\n"
    "  yolov5s.hef       - 16 MB, COCO pretrained, FAR range specialist\n"
    "  yolov7.hef        - 55 MB, COCO pretrained, MID range specialist\n"
    "  yolov8s-coco.hef  - 35 MB, COCO pretrained, CLOSE range specialist\n"
    "\n"
    "Detection: COCO class 39 (bottle) - all orientations (standing, lying, tilted)\n"
    "Filtering: Bottles held by humans (COCO class 0) are excluded.\n"
    "Toggle live: press 5/7/8/A during camera view to switch models."
)

pdf.sub_title("Sensor Fusion")
pdf.body_text(
    "The navigator fuses two data sources for 360-degree awareness:\n\n"
    "1. CAMERA (Raspberry Pi + Hailo-8):\n"
    "   - Detects bottles (target) and persons (avoid)\n"
    "   - Estimates distance by bounding box fill ratio\n"
    "   - Guides approach angle to center bottle in frame\n\n"
    "2. ULTRASONIC (ESP32, via /sensor endpoint):\n"
    "   - Left and right distance readings (cm)\n"
    "   - Detects walls, furniture, obstacles camera cannot see\n"
    "   - Polled at 10 Hz over WiFi HTTP\n\n"
    "Decision priority: Ultrasonic STOP > Camera person avoid > Bottle approach > Roam"
)

# ============================================================
# PAGE 10: Command Reference
# ============================================================
pdf.add_page()
pdf.section_title("10. Command Reference (Serial + WiFi)")

pdf.body_text(
    "USB Serial: 115200 baud  |  WiFi AP: PetBottle_Robot (petbottle123)\n"
    "Web UI: http://192.168.4.1  |  API: /cmd?c=<CMD>  |  Sensors: /sensor"
)

w6 = [40, 150]
pdf.sub_title("Drive Wheels")
pdf.table_header(["Command", "Description"], w6)
cmds = [
    ("F<speed>", "Drive forward (0-255, default 80)"),
    ("J<speed>", "Drive backward (0-255)"),
    ("TL<speed>", "Turn left (left reverse, right forward)"),
    ("TR<speed>", "Turn right (left forward, right reverse)"),
    ("W<l>,<r>", "Raw wheel control (-255 to 255). E.g. W100,-50"),
    ("X", "Stop both wheels immediately"),
]
for c in cmds:
    pdf.table_row(list(c), w6, mono_first=True)

pdf.ln(2)
pdf.sub_title("Base / Lift / Servos")
pdf.table_header(["Command", "Description"], w6)
cmds2 = [
    ("B<angle>", "Rotate base (-270 to +270 deg)"),
    ("BH", "Home base to 0 degrees"),
    ("U<speed>", "Lift arm UP (0-255)"),
    ("D<speed>", "Lift arm DOWN (0-255)"),
    ("S", "Stop lift motor"),
    ("L<angle>", "Left servo (0-180 deg)"),
    ("R<angle>", "Right servo (0-180 deg)"),
    ("O", "Both servos open (0 deg)"),
    ("C", "Both servos close (90 deg)"),
]
for c in cmds2:
    pdf.table_row(list(c), w6, mono_first=True)

pdf.ln(2)
pdf.sub_title("System")
pdf.table_header(["Command", "Description"], w6)
cmds3 = [
    ("P", "Full pickup sequence (lower, grab, lift, rotate, drop, home)"),
    ("H", "Home all motors (wheels stop, lift up, servos open, base 0)"),
    ("E", "EMERGENCY STOP - all motors off, EN pins LOW"),
    ("?", "Show help and current status"),
]
for c in cmds3:
    pdf.table_row(list(c), w6, mono_first=True)

# ============================================================
# PAGE 11: Pickup Sequence + Safety
# ============================================================
pdf.add_page()
pdf.sub_title("Pickup Sequence (Command: P)")
pdf.body_text(
    "1. Stop wheels, open both servos\n"
    "2. Lower arm (speed 150, 3 seconds)\n"
    "3. Close both servos (grab bottle)\n"
    "4. Lift arm (speed 200, 3 seconds)\n"
    "5. Rotate base to 180 degrees (bin on top)\n"
    "6. Lower slightly (speed 120, 1 second)\n"
    "7. Open both servos (release bottle into bin)\n"
    "8. Lift arm back up (speed 200, 1.5 seconds)\n"
    "9. Return to home position (base 0, servos open)\n"
    "\n"
    "Triggered automatically by navigator.py when bottle is centered\n"
    "and close enough (fills >15% of camera frame)."
)

pdf.section_title("11. Safety Notes")
pdf.body_text(
    "BOOT SAFETY:\n"
    "- GPIO2 (Right RPWM) has built-in pull-down: stays LOW at boot.\n"
    "- All EN pins start LOW (drivers disabled) until PWM is zeroed.\n"
    "- Default speed: 80/255 (~31%) for safe testing.\n"
    "\n"
    "FUSES: Add 10A fuse on each BTS7960B B+ line.\n"
    "\n"
    "ULTRASONIC SAFETY:\n"
    "- Add voltage divider (1K+2K) on ECHO lines for 5V->3.3V.\n"
    "- 15ms delay between left/right readings prevents crosstalk.\n"
    "- Timeout: 30ms (30000us) prevents blocking on no echo.\n"
    "\n"
    "NAVIGATOR SAFETY:\n"
    "- Emergency stop if ultrasonic < 15cm on either side.\n"
    "- Camera detects persons - robot avoids if person fills >20% of frame.\n"
    "- Pickup sequence stops wheels first, then operates arm.\n"
    "- Lost WiFi: robot keeps last known sensor data, stops on timeout.\n"
    "\n"
    "EMERGENCY STOP (E): Disables all motor drivers immediately.\n"
    "Send 'H' to re-enable and home all axes.\n"
    "\n"
    "GROUND RULE: ALL GNDs must be tied together - battery, DC-DC,\n"
    "ESP32, BTS7960B (x3), L298N, servos, LCD, buzzer, HC-SR04 (x2)."
)

# ============================================================
# PAGE 12: Bill of Materials
# ============================================================
pdf.add_page()
pdf.section_title("12. Bill of Materials")
w7 = [15, 80, 45, 50]
pdf.table_header(["Qty", "Component", "Model", "Purpose"], w7)
bom = [
    ("1", "ESP32 Dev Board", "ESP32-D0WD", "Motor/sensor controller"),
    ("1", "Raspberry Pi 5", "RPi 5 4/8GB", "Vision + navigation brain"),
    ("1", "Hailo-8 NPU", "M.2 module", "YOLO AI accelerator"),
    ("1", "Pi Camera", "V2 or V3", "Bottle/obstacle detection"),
    ("3", "BTS7960B Motor Driver", "HW-039", "43A H-bridge"),
    ("1", "L298N Motor Driver", "L298N", "Stepper driver"),
    ("2", "HC-SR04 Ultrasonic", "HC-SR04", "Obstacle distance"),
    ("2", "24V DC Motor", "-", "Drive wheels (L/R)"),
    ("1", "DC Geared Motor", "58SW31ZY", "Arm lift"),
    ("1", "Stepper Motor", "NEMA 17", "Base rotation"),
    ("2", "Servo Motor", "MG996R", "Left/Right arm grab"),
    ("1", "LCD I2C Display", "16x2 PCF8574", "Status display"),
    ("1", "Active Buzzer", "-", "Audio alerts"),
    ("1", "Lithium Battery", "24V 30Ah", "Main power"),
    ("1", "DC-DC Converter", "24V->5V 5A", "Logic/servo power"),
    ("3", "Blade Fuse + Holder", "10A", "Driver protection"),
    ("2", "Resistor 1K ohm", "-", "Ultrasonic voltage divider"),
    ("2", "Resistor 2K ohm", "-", "Ultrasonic voltage divider"),
]
for r in bom:
    pdf.table_row(list(r), w7, mono_first=False)

pdf.ln(4)
pdf.sub_title("Software Files")
w_sw = [60, 130]
pdf.table_header(["File", "Description"], w_sw)
sw_rows = [
    ("robotic_arm.ino", "ESP32 firmware: motors, sensors, WiFi AP, /sensor API"),
    ("camera.py", "YOLO v5/v7/v8 model stitching, detection, postprocessing"),
    ("navigator.py", "Autonomous brain: roam, avoid, approach, pickup"),
    ("server.py", "OTA update server for remote code deployment"),
    ("yolov5s.hef", "YOLOv5s COCO model (Hailo HEF, 16MB)"),
    ("yolov7.hef", "YOLOv7 COCO model (Hailo HEF, 55MB)"),
    ("yolov8s-coco.hef", "YOLOv8s COCO model (Hailo HEF, 35MB)"),
]
for r in sw_rows:
    pdf.table_row(list(r), w_sw, mono_first=True)


# Save
output_path = r"c:\Users\FlorenceSangrenes\Downloads\florence\emathrix\pet bottle\petbottle\PetBottle_Robotic_Arm_Schematic.pdf"
pdf.output(output_path)
print(f"PDF saved to: {output_path}")
