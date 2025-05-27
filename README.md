# Inverse Kinematics on Arduino: 6 DoF Robotic Arm

This project implements inverse kinematics for a 6 degrees-of-freedom (6 DoF) robotic arm using an Arduino Uno, MG996R servo motors, and a PCA9685 PWM driver. The arm accepts 3D Cartesian coordinates and a desired wrist tilt angle, and computes joint angles to reach the target position while aligning the wrist with a global Z-axis reference.
The project introduction and documentation can be found under the docs folder.

---

## 📁 Project Structure

```
Inverse_Kinematics_Arduino_6DoF/
├── Inverse_Kinematics_Arduino_6DoF.ino       # Main Arduino sketch
├── ik_solver.h / ik_solver.cpp               # Inverse kinematics: 3D positioning and wrist tilt
├── servo_driver.h / servo_driver.cpp         # PCA9685-based servo control
└── interface.h / interface.cpp               # Serial command parsing
```

---

## 🩾 Hardware Used

- 6x MG996R Servo Motors  
- PCA9685 16-Channel PWM Driver  
- Arduino Uno  
- External 5V Power Supply (for servos)  
- 6 DoF Robotic Arm Frame (custom or kit-based)

---

## 💡 Features

- Supports inverse kinematics for the following 4 joints:
  - Base rotation (shoulder rotate)
  - Shoulder tilt
  - Elbow
  - Wrist tilt (relative to global Z-axis)
- Accepts absolute Cartesian commands via Serial interface
- Modular architecture in C++ for easy scaling and debugging
- Joint angle limits and link offsets included in the model

---

## 🔧 How to Use

1. Clone or download this repository.  
2. Ensure the folder name matches the `.ino` filename.  
3. Open the `.ino` file in the Arduino IDE.  
4. Install the required library:
   - `Adafruit PWM Servo Driver Library`  
5. Upload the sketch to your Arduino Uno.  
6. Open the Serial Monitor or send commands over USB. Example commands:

```
GOTO X=150 Y=0 Z=120
GOTO X=150 Y=0 Z=120 TILT=90 ROTATE=90
GRIP OPEN
GRIP CLOSE
```

Where:
- `X`, `Y`, `Z` define the target position in mm. 
- `TILT` defines the desired wrist tilt angle in degrees. Optional parameter. Default is 90 degrees.
  - `90` = horizontal (parallel to surface).
  - `0` = vertical downward.
  - `180` = vertical upward.
- `ROTATE` defines the desired wrist rotate angle in degrees. Optional parameter. Default is 90 degrees.

---

## 🧲 IK Solver Details

- The IK model includes link lengths and joint offsets:
  - `L1`: Height from base to shoulder rotate (85 mm)
  - `L2`: Offset from rotate axis to shoulder tilt axis (23 mm)
  - `L3`: Upper arm length (105 mm)
  - `L4`: Forearm length (150 mm)
  - `L5`: Wrist-to-gripper length (180 mm)
- All coordinates are interpreted in a right-handed 3D system where the base is at the origin.
- The wrist tilt angle is calculated to match a user-defined approach angle with respect to the global Z-axis.

---

## 🧬 License

MIT License — free to use, modify, and distribute.
