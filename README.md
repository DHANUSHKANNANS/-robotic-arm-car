# 🤖 Mobile Robotic Arm Car — Pick & Place via Bluetooth

A 4-wheel Arduino-based robotic car with a 4-DOF servo arm, controlled wirelessly from a smartphone via Bluetooth for pick-and-place operations.


---

## 🔧 Hardware

| Component           | Details                          |
|---------------------|----------------------------------|
| Microcontroller     | Arduino Uno / Nano               |
| Chassis             | 4-wheel DC motor car kit         |
| Motor Driver        | L298N Dual H-Bridge              |
| Arm Joints          | 3× SG90 / MG996R Servo Motors   |
| Gripper             | 1× SG90 Servo Motor              |
| Bluetooth Module    | HC-05 or HC-06                   |
| Power               | 7.4V LiPo or 4× AA batteries    |
| Frame               | Aluminium bracket arm kit        |

---

## 📌 Wiring

### Motor Driver (L298N)
```
ENA → D5    IN1 → D4    IN2 → D3
ENB → D6    IN3 → D7    IN4 → D8
```

### Servo Motors
```
Base Servo     → D9
Shoulder Servo → D10
Elbow Servo    → D11
Gripper Servo  → D12
```

### Bluetooth HC-05
```
HC-05 TX → Arduino D0 (RX)
HC-05 RX → Arduino D1 (TX) [use 1kΩ+2kΩ voltage divider]
HC-05 VCC → 5V
HC-05 GND → GND
```

> ⚠️ **Note:** Disconnect HC-05 before uploading code (shares Serial pins).

---

## 🚀 Getting Started

1. Clone this repo:
   ```bash
   git clone https://github.com/YOUR_USERNAME/robotic-arm-car.git
   ```
2. Open `src/robotic_arm_car.ino` in Arduino IDE
3. Install library: `Servo.h` (built-in)
4. Disconnect Bluetooth module, upload code, reconnect
5. Pair HC-05 (default PIN: `1234`)
6. Use a Bluetooth terminal app and send commands (see below)

---

## 📱 Bluetooth Commands

| Key | Action              |
|-----|---------------------|
| `F` | Forward             |
| `B` | Backward            |
| `L` | Turn Left           |
| `R` | Turn Right          |
| `S` | Stop                |
| `q` | Base → Clockwise    |
| `a` | Base → Counter-CW   |
| `w` | Shoulder Up         |
| `s` | Shoulder Down       |
| `e` | Elbow Up            |
| `d` | Elbow Down          |
| `o` | Gripper Open        |
| `c` | Gripper Close       |
| `P` | Auto Pick & Place   |
| `H` | Return Home         |

---

## 🧮 Kinematics

See [`docs/KINEMATICS.md`](docs/KINEMATICS.md) for:
- Forward Kinematics equations
- Inverse Kinematics Python implementation
- Pre-computed waypoint table
- Servo calibration guide

---

## 📁 Project Structure

```
robotic-arm-car/
├── src/
│   └── robotic_arm_car.ino    # Main Arduino sketch
├── docs/
│   ├── KINEMATICS.md          # FK/IK math + waypoints
│   └── robot.jpg              # Robot photo
├── schematics/
│   └── wiring_diagram.png     # (add your own)
└── README.md
```

---

## 🔮 Future Improvements

- [ ] Add ultrasonic sensor (HC-SR04) for obstacle avoidance
- [ ] PID speed control for smoother chassis movement
- [ ] Custom Android app with joystick UI
- [ ] Object detection with ESP32-CAM
- [ ] EEPROM-based sequence recording/playback

---

## 📄 License

MIT License — feel free to use and modify.
