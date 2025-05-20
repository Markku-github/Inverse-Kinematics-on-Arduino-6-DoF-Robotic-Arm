# Inverse Kinematics on Arduino: 6 DoF Robotic Arm

This project demonstrates inverse kinematics control of a 6 degrees-of-freedom (6 DoF) robotic arm using an Arduino Uno, MG996R servo motors, and a PCA9685 PWM driver. The arm receives 3D Cartesian coordinates via serial commands and translates them into joint angles using basic IK calculations.

---
## 📁 Project Structure

```
Inverse_Kinematics_Arduino_6DoF/
├── Inverse_Kinematics_Arduino_6DoF.ino       # Main Arduino sketch
├── ik_solver.h / ik_solver.cpp               # Inverse kinematics calculations
├── servo_driver.h / servo_driver.cpp         # Servo motor control
└── interface.h / interface.cpp               # Serial command parsing
```

---

## 🩾 Hardware Used

- 6x MG996R Servo Motors
- PCA9685 16-Channel PWM Driver
- Arduino Uno
- External 5V Power Supply (for servos)
- 6 DoF Robotic Arm Frame

---

## 💡 Features

- Calculates and executes inverse kinematics for a 3-joint planar section of the arm.
- Accepts serial commands for positioning and gripper control.
- Modular C++ code structure for easy expansion.

---

## 🔧 How to Use

1. Clone or download this repository.
2. Ensure the `.ino` file name matches the folder name.
3. Open the project in the Arduino IDE.
4. Install required libraries:
   - `Adafruit PWM Servo Driver Library`
5. Upload the code to your Arduino Uno.
6. Connect to the Arduino serial monitor and send commands like:

```bash
GOTO X=100 Y=50 Z=50
GRIP OPEN
GRIP CLOSE
```

---

## 🧬 License

MIT License. You are free to use, modify, and distribute.