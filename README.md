

# High-Performance PID Line Follower Robot (ESP32 & Arduino Nano Versions)

This repository contains the code and documentation for building a fast and efficient line-following robot using a PID (Proportional-Integral-Derivative) control algorithm. The project is provided in two versions:

### 1\. Project Overview
This project demonstrates how to build a high-performance line-following robot. Instead of using basic `if-else` logic that results in wobbly movement, this robot employs a PID controller. This allows it to dynamically calculate the precise amount of correction needed to stay on the line, resulting in smoother turns and higher possible speeds.

The robot uses a 5-channel infrared (IR) sensor array to detect the line, an L298N motor driver to control two DC motors, and either an Arduino Nano or an ESP32 as its brain.

### 2\. How PID Control Works

The core of this robot is the PID algorithm. It works by continuously calculating an "error" value, which represents how far the robot is from the center of the line. It then calculates a correction based on three terms [5, 6, 7]:

  * **Proportional (P):** This term reacts to the **current error**. If the robot is far from the line, the correction is large. If it's only slightly off, the correction is small. This provides the primary steering force but can cause wobbling on its own.
  * **Integral (I):** This term looks at the **sum of past errors**. If the robot has a small, consistent drift to one side (due to mechanical imbalances), the integral term will build up over time and apply a counteracting force to bring the robot back to the exact center.
  * **Derivative (D):** This term looks at the **rate of change of the error**, essentially predicting future errors. As the robot turns back towards the line, the error decreases rapidly. The D-term anticipates this and dampens the correction to prevent the robot from overshooting the line and oscillating. This is what makes the movement smooth.

The final correction is a weighted sum of these three terms: `Correction = (Kp * P) + (Ki * I) + (Kd * D)`. Our job is to find the perfect `Kp`, `Ki`, and `Kd` values for our specific robot.




Here’s a **super-detailed, comprehensive README.md** for your GitHub repository, covering all four codes, hardware connections, setup, operation, PID tuning, and troubleshooting. This version is designed to be a **one-stop guide** for anyone using your project, with clear tables, step-by-step instructions, and in-depth explanations.

---

## **2. Code 1: Basic Line Follower (Arduino Nano)**

### **Purpose**
A **simple, rule-based line-following robot** using two IR sensors. Ideal for beginners to understand the basics of line following without PID.

### **Features**
- Uses **two IR sensors** (left and right).
- **Basic logic**: Move forward if both sensors are on the line, turn left/right if one sensor is off.
- **No PID, no calibration** required.
- **Easy to build and understand**.

### **Hardware Requirements**
| Component                | Quantity | Notes                                  |
|--------------------------|----------|----------------------------------------|
| Arduino Nano             | 1        |                                        |
| L298N Motor Driver       | 1        | Or TB6612FNG                           |
| DC Motors (6V)           | 2        | With wheels                            |
| IR Sensor Module         | 2        | Digital output (e.g., TCRT5000)        |
| Chassis                  | 1        | Any lightweight 2WD robot chassis      |
| 7.4V Li-ion Battery      | 1        | Or 6x AA battery pack                  |
| Jumper Wires             | ~20      |                                        |
| Breadboard (optional)    | 1        | For prototyping                        |

### **Wiring Diagram**
| Component         | Arduino Nano Pin |
|-------------------|------------------|
| IR Sensor (Left)  | A0               |
| IR Sensor (Right) | A1               |
| Motor A IN1       | 2                |
| Motor A IN2       | 3                |
| Motor A PWM       | 6                |
| Motor B IN1       | 4                |
| Motor B IN2       | 5                |
| Motor B PWM       | 9                |
| Motor Driver STBY | 7                |
| Motor Driver GND  | GND              |
| Motor Driver VCC  | 5V (Logic)       |
| Motor Power       | 7.4V (Battery)   |

### **Setup & Operation**
1. **Assemble the robot** and wire all components as per the table.
2. **Upload the code** to your Arduino Nano using the Arduino IDE.
3. **Place the robot** on a black line with a white background.
4. **Power on** the robot. It will start following the line using simple logic.

### **Code Explanation**
- **IR Sensor Logic**:
  - `0` = Black line detected
  - `1` = White surface detected
- **Motor Control**:
  - `moveForward()`: Both motors move forward.
  - `turnLeft()`: Left motor reverses, right motor moves forward.
  - `turnRight()`: Right motor reverses, left motor moves forward.
  - `stopMotors()`: Both motors stop.

---

## **3. Code 2: PID Line Follower with Web Server (ESP32)**

### **Purpose**
An **advanced line-following robot** using **PID control** and a **web interface** for real-time tuning and monitoring.

### **Features**
- **PID control** for smooth and precise line following.
- **Web server** for real-time PID tuning, motor control, and sensor calibration.
- **8-channel IR sensor array** for precise line detection.
- **State machine** for handling line loss and intersections.

### **Hardware Requirements**
| Component                | Quantity | Notes                                  |
|--------------------------|----------|----------------------------------------|
| ESP32 Dev Board          | 1        |                                        |
| TB6612FNG Motor Driver   | 1        |                                        |
| DC Motors (6V)           | 2        | With wheels                            |
| 8-Channel IR Sensor Array| 1        | (e.g., Pololu QTR-8A)                  |
| Chassis                  | 1        | Any lightweight 2WD robot chassis      |
| 7.4V Li-ion Battery      | 1        | Or 6x AA battery pack                  |
| Jumper Wires             | ~30      |                                        |

### **Wiring Diagram**
| Component         | ESP32 Pin |
|-------------------|-----------|
| Motor A IN1       | 18        |
| Motor A IN2       | 5         |
| Motor A PWM       | 19        |
| Motor B IN1       | 4         |
| Motor B IN2       | 15        |
| Motor B PWM       | 21        |
| Motor Driver STBY | 2         |
| IR Sensor Array   | 26, 39, 36, 35, 34, 33, 32, 25 |




### **Setup & Operation**

1.  **Assemble Hardware:** Build your robot and wire all components according to the table above.
2.  **Install Libraries:** In the Arduino IDE, go to **Sketch \> Include Library \> Manage Libraries...** and install the following:
      * `QTRSensors` by Pololu
      * `L298N` (ensure it's a library that matches the `L298N motor1(PWMA, AIN1, AIN2);` constructor format).
3.  **Upload Code:**
      * Open the ESP32 code (provided below) in the Arduino IDE.
      * Go to **Tools \> Board** and select your ESP32 model (e.g., "ESP32 Dev Module").
      * Select the correct **Port** and click **Upload**.
4. **Update WiFi credentials** in the code (`ssid` and `password`).
5. **Power on** the ESP32 and connect to the same WiFi network.
6. **Open the web interface** at the ESP32’s IP address (printed in the Serial Monitor).
7. **Use the web UI** to calibrate sensors, tune PID, and control the robot.
8.  **Sensor Calibration (Physical Step):**
      * After uploading, the onboard LED will light up. This signals the start of the 10-second sensor calibration phase.
      * During these 10 seconds, you must **physically slide the robot's sensor array back and forth across the black line**. This allows the sensors to learn the min/max values for black and white. This step is crucial and must be done first.
9.  **PID Tuning & Control via Bluetooth:**
      * On your smartphone, install a Bluetooth Serial Terminal app (e.g., "Serial Bluetooth Terminal" for Android).
      * Power on your robot. The ESP32 will start a Bluetooth serial device.
      * In your phone's Bluetooth settings, pair with the new device (its name will be the one set in `SerialBT.begin()`).
      * Open the Bluetooth terminal app and connect to the paired device.
      * You can now send commands to tune the PID constants and control the robot. The code uses a 2-byte protocol: the first byte identifies the parameter, and the second is the value.
          * **Start/Stop Robot:** Send `7,1` to start, `7,0` to stop.
          * **Set Kp:** Send `1, [value]` (e.g., `1,10` sets Kp to 10).
          * **Set Ki:** Send `3, [value]` (e.g., `3,5` sets Ki to 5).
          * **Set Kd:** Send `5, [value]` (e.g., `5,15` sets Kd to 15).
          * **Set Multipliers:** The code includes multipliers to allow for fractional PID values. For example, to set `Kp` to `0.8`, you would send `1,8` (for Kp) and `2,1` (for multiP, which means divide by $10^1$).

### **Web Interface Guide**
- **PID Tuning**: Adjust `Kp`, `Ki`, and `Kd` in real-time.
- **Motor Control**: Enable/disable motors.
- **Sensor Calibration**: Calibrate sensors for 5 seconds.
- **Status Console**: Monitor robot status and sensor values.

### **Code Explanation**
- **PID Control**:
  - `Kp`: Proportional gain (reacts to current error).
  - `Ki`: Integral gain (corrects steady-state error).
  - `Kd`: Derivative gain (dampens oscillations).
- **State Machine**:
  - `FOLLOW_LINE`: Normal operation.
  - `SEARCH_LEFT/SEARCH_RIGHT`: If the line is lost.
  - `AT_INTERSECTION`: If all sensors detect black (intersection).
- **Sensor Calibration**: Automatically records min/max values for each sensor.

---

## **4. Code 3: PID Line Follower (ESP32, No Web Server)**

### **Purpose**
A **standalone PID line follower** with **manual calibration**, no web server.

### **Features**
- **PID control** for smooth line following.
- **Manual sensor calibration** using the BOOT button.
- **8-channel IR sensor array**.
- **State machine** for handling line loss and intersections.

### **Hardware Requirements**
| Component                | Quantity | Notes                                  |
|--------------------------|----------|----------------------------------------|
| ESP32 Dev Board          | 1        |                                        |
| TB6612FNG Motor Driver   | 1        |                                        |
| DC Motors (6V)           | 2        | With wheels                            |
| 8-Channel IR Sensor Array| 1        | (e.g., Pololu QTR-8A)                  |
| Chassis                  | 1        | Any lightweight 2WD robot chassis      |
| 7.4V Li-ion Battery      | 1        | Or 6x AA battery pack                  |

### **Wiring Diagram**
| Component         | ESP32 Pin |
|-------------------|-----------|
| Motor A IN1       | 18        |
| Motor A IN2       | 5         |
| Motor A PWM       | 19        |
| Motor B IN1       | 4         |
| Motor B IN2       | 15        |
| Motor B PWM       | 21        |
| Motor Driver STBY | 2         |
| IR Sensor Array   | 26, 39, 36, 35, 34, 33, 32, 25 |
| BOOT Button       | 0         |

### **Setup & Operation**
1. **Upload the code** to your ESP32.
2. **Press the BOOT button** to start calibration.
3. **Move the robot** over white and black surfaces to calibrate sensors.
4. **The robot will start** following the line using PID control.

### **Code Explanation**
- **PID Control**: Same as Code 2.
- **Manual Calibration**:
  - Press BOOT button to start.
  - Move robot over white and black surfaces.
  - Press BOOT button again to finish.
- **State Machine**: Same as Code 2.

---

## **5. Code 4: Maze Solver with Web Interface (ESP32)**

### **Purpose**
A **maze-solving robot** that records and replays the shortest path, with a **web interface** for control and monitoring.

### **Features**
- **Right-hand rule maze solving**.
- **Records path** during dry run, optimizes, and replays the shortest path.
- **Web interface** for real-time control, speed/timing adjustments, and monitoring.
- **8-channel IR sensor array** for precise wall detection.

### **Hardware Requirements**
| Component                | Quantity | Notes                                  |
|--------------------------|----------|----------------------------------------|
| ESP32 Dev Board          | 1        |                                        |
| TB6612FNG Motor Driver   | 1        |                                        |
| DC Motors (6V)           | 2        | With wheels                            |
| 8-Channel IR Sensor Array| 1        | (e.g., Pololu QTR-8A)                  |
| Chassis                  | 1        | Any lightweight 2WD robot chassis      |
| 7.4V Li-ion Battery      | 1        | Or 6x AA battery pack                  |

### **Wiring Diagram**
| Component         | ESP32 Pin |
|-------------------|-----------|
| Motor A IN1       | 18        |
| Motor A IN2       | 5         |
| Motor A PWM       | 19        |
| Motor B IN1       | 4         |
| Motor B IN2       | 15        |
| Motor B PWM       | 21        |
| Motor Driver STBY | 2         |
| IR Sensor Array   | 25, 32, 33, 34, 35, 36, 39, 26 |

### **Setup & Operation**
1. **Upload the code** to your ESP32.
2. **Update WiFi credentials** in the code (`ssid` and `password`).
3. **Power on** the ESP32 and connect to the same WiFi network.
4. **Open the web interface** at the ESP32’s IP address.
5. **Use the web UI** to calibrate sensors, start dry run, and replay the shortest path.

### **Web Interface Guide**
- **Main Controls**: Calibrate, Dry Run, Stop.
- **Speed Settings**: Adjust forward, turn, and fast speeds.
- **Timing Settings**: Adjust turn and back-off times.
- **Robot Status**: Monitor mode, path length, and current path.
- **Serial Monitor**: Real-time logs.

### **Code Explanation**
- **Maze Solving**:
  - **Dry Run**: Records path using right-hand rule.
  - **Path Optimization**: Simplifies path (e.g., "LRL" → "B").
  - **Fast Run**: Replays the optimized path at high speed.
- **Sensor Logic**:
  - `rightAvailable()`, `forwardAvailable()`, `leftAvailable()`: Detects possible moves.
  - `areAllBlack()`: Detects goal (all sensors on black).

---

## **6. Hardware Connections (Detailed Pinouts)**

### **Motor Driver (TB6612FNG)**
| Motor Driver Pin | ESP32 Pin | Arduino Nano Pin | Function         |
|------------------|-----------|------------------|------------------|
| AIN1             | 18        | 2                | Motor A Direction|
| AIN2             | 5         | 3                | Motor A Direction|
| PWMA             | 19        | 6                | Motor A Speed    |
| BIN1             | 4         | 4                | Motor B Direction|
| BIN2             | 15        | 5                | Motor B Direction|
| PWMB             | 21        | 9                | Motor B Speed    |
| STBY             | 2         | 7                | Standby          |
| VM               | -         | -                | Motor Power (7.4V)|
| VCC              | 5V        | 5V               | Logic Power      |
| GND              | GND       | GND              | Ground           |

### **IR Sensor Array**
| Sensor Pin | ESP32 Pin | Arduino Nano Pin | Function         |
|------------|-----------|------------------|------------------|
| Sensor 1   | 26        | A0               | Leftmost Sensor  |
| Sensor 2   | 39        | A1               |                  |
| Sensor 3   | 36        | A2               |                  |
| Sensor 4   | 35        | A3               |                  |
| Sensor 5   | 34        | A4               |                  |
| Sensor 6   | 33        | -                |                  |
| Sensor 7   | 32        | -                |                  |
| Sensor 8   | 25        | -                | Rightmost Sensor |

---

## **7. PID Tuning: The Complete Guide**

### PID Control Formulas

**Complete PID:**
```
PID_output = (Kp × Error) + (Ki × Σ Error) + (Kd × ΔError/Δt)
```

**Discrete Implementation:**
```
Error = Setpoint - Current_Position
P = Error
I = I + Error
D = Error - Previous_Error
PID_value = (Kp × P) + (Ki × I) + (Kd × D)
```

**Motor Control:**
```
Left_Motor_Speed = Base_Speed + PID_value
Right_Motor_Speed = Base_Speed - PID_value
```

### Initial Kp Calculation

```
Max_Error = Sensor_Range / 2
Kp = Max_Motor_Speed / Max_Error
```

**Example:**
```
Sensor_Range = 7000 (for 8-sensor array, 0-7000)
Max_Error = 3500
Max_Motor_Speed = 100
Kp = 100 / 3500 = 0.028
```

### Line Position Calculation

**Weighted Average Method:**
```
Position = (Σ(Sensor_Value[i] × i × 1000)) / Σ(Sensor_Value[i])
```

**Where:**
- i = sensor index (0 to 7)
- Sensor_Value[i] = normalized value (0-1000) from sensor i

---

### **Step-by-Step Tuning**
1. **Set Ki and Kd to 0.**
2. **Increase Kp** until the robot follows the line but starts to oscillate.
3. **Increase Kd** to reduce oscillations and smooth turns.
4. **Add a small Ki** if the robot drifts consistently to one side.



| Parameter | Start Value | Typical Range | Notes                          |
|-----------|-------------|---------------|--------------------------------|
| Kp        | 0.01        | 0.01–0.1      | Too high → oscillations        |
| Ki        | 0.0001      | 0.0001–0.001  | Too high → instability         |
| Kd        | 0.0005      | 0.0005–0.01   | Too high → slow response       |

### **Tuning Tips**
- **Oscillations?** Reduce Kp or increase Kd.
- **Slow response?** Increase Kp.
- **Steady drift?** Increase Ki slightly.

---

## **8. Troubleshooting & FAQ**

### **Common Issues**
| Issue                     | Solution                                  |
|---------------------------|-------------------------------------------|
| Robot doesn’t move        | Check motor connections and power supply. |
| Sensors not reading       | Verify IR sensor wiring and calibration.  |
| Web interface not loading | Check WiFi credentials and IP address.    |
| Robot oscillates          | Reduce Kp or increase Kd.                 |
| Line lost frequently      | Increase base speed or adjust thresholds. |

### **FAQ**
**Q: How do I find the ESP32’s IP address?**
A: Open the Serial Monitor after uploading the code. The IP address will be printed.

**Q: Can I use a different motor driver?**
A: Yes, but update the pin definitions in the code.

**Q: How do I change the PID values in Code 1?**
A: Code 1 doesn’t use PID. For PID, use Code 2 or 3.

---


**Ready to build your robot?** Follow the guides above, and happy tinkering! 🤖🚀
**Questions?** Open an issue or contact me directly.
-----


