# SmartDrive Assist

**Repository:** [GitHub Repo Link](https://github.com/KirolousFouty/SmartDrive_Assist)

## 1. Project Overview
**SmartDrive Assist** is an embedded system designed to provide intuitive and safe motion control for the Dagu Wild Thumper robot platform. It targets assistive mobility applications (e.g., smart wheelchairs) by combining proportional joystick control, speed blending, and active safety monitoring.

Due to the complex I/O requirements, the system utilizes a **Dual-MCU Architecture**:
* **STM32L432KC (Master):** Handles sensor data acquisition (Joystick, Ultrasonic), user input (Keypad), safety logic, and inverse kinematics.
* **ESP32 (Slave/Bridge):** Acts as an intelligent I/O expander, driving the LCD and interfacing with the robot's high-power motor driver via a secondary UART channel.

### Team Members
- **Aly Mahmoud** ([@AlyMohamedd](https://github.com/AlyMohamedd))
- **Kirolous Fouty** ([@KirolousFouty](https://github.com/KirolousFouty))
- **Yussuf Abdelrahman** ([@Yussuf12345](https://github.com/Yussuf12345))

---

## 2. System Architecture

### Hardware Components
* **Master Controller:** STM32L432KC (Nucleo-32)
* **Peripheral Controller:** ESP32
* **Input Devices:**
    * 2-Axis Analog Joystick (X/Y control)
    * 4x4 Matrix Keypad (Mode/Speed selection)
* **Sensors:** HC-SR04 Ultrasonic Distance Sensor
* **Output/Feedback:**
    * Digilent PmodCLP (16x2 LCD) via ESP32
    * Status LEDs (Onboard + External)
* **Actuation:** Dagu Wild Thumper 4WD Chassis + Dagu TReX Motor Controller

### Communication Flow
The system relies on a custom UART protocol to synchronize the two microcontrollers:
1.  **STM32** reads Joystick (ADC) and Keypad (GPIO Scanning).
2.  **STM32** calculates tank-drive mixing, speed scaling, and safety constraints.
3.  **STM32** sends packetized commands to the ESP32 via UART (115200 baud).
4.  **ESP32** parses packets to:
    * Update the LCD with real-time speed, mode, and distance.
    * Forward motor commands to the Dagu Controller via a separate UART line (19200 baud).

### Schematic Block Diagram

* Author-generated schematic (Nano Banana Pro).
<img src="https://github.com/user-attachments/assets/99b8f7c4-ddb7-4951-96fb-5ff1b98046bd" width="2816" alt="Schematic Block Diagram">


---

## 3. Features & Implementation

### 3.1 Advanced Motion Control
* **Proportional Tank Drive:** Maps polar joystick coordinates (Angle/Magnitude) to differential wheel speeds.
* **Precision Mode:** Toggled via Keypad (`#`), reduces motor output scaling by 50% for fine maneuvers.
* **Spin Assist:** Dedicated buttons (`A`/`*`) for in-place rotations.
* **Pre-defined Trips:** Automated path navigation triggered via Keypad (`B`).

### 3.2 Safety Systems
* **Collision Avoidance:**
    * *Zone 1 (>20cm):* Normal Operation.
    * *Zone 2 (10-20cm):* **Warning Mode.** System alerts user.
    * *Zone 3 (<10cm):* **Emergency Stop.** Forward motion physically blocked; reverse/turn allowed for recovery.
* **Deadman Switch:** Motors automatically zero if joystick returns to neutral.
* **Flash Calibration:** Joystick center/min/max values are calibrated and stored in the STM32 Flash memory preventing joystick drift.

### 3.3 User Interface
* **Keypad Mapping:**
    * `0-9`: Set global speed limit (0% to 100%).
    * `A`: Spin Right.
    * `B`: Toggle Trip.
    * `C`: Abort any active autonomous trip.
    * `D`: Recalibrate Joystick.
    * `#`: Toggle Precision Mode.

* **LCD Dashboard:** Displays connectivity status, current speed %, active mode (Forward/Reverse/Spin), and distance to obstacles.

---

## 4. Technical Challenges & Solutions

### ⚠ Challenge 1: Insufficient GPIO Pins
The STM32L432KC is a compact Nucleo-32 board. After connecting the 4x4 Keypad (8 pins), Joystick (2 ADC), and Ultrasonic sensor (2 GPIO), we lacked sufficient pins to drive the parallel LCD interface (6+ pins) and the specific UART requirements of the Motor Driver.
* **Solution:** We introduced an **ESP32** as a dedicated "Bridge". The STM32 offloads display rendering and motor communication to the ESP32. This distributed approach solved the pin constraint and separated logic (STM32) from driver interfacing (ESP32).

### ⚠ Challenge 2: Analog Joystick Drift
The analog potentiometers in the joystick suffered from significant center-value drift and mechanical "jitter", causing the robot to creep move and jitter when idle.
* **Solution:** We implemented a robust calibration routine (`Calibrate_Joystick` in `main.c`).
    1.  The system samples the center position at startup.
    2.  It calculates a dynamic "deadzone" (approx. 15%).
    3.  Calibration data is written to the **STM32 Flash Memory**.
    4.  On boot, the system validates the Flash checksum; if valid, it loads the calibration; otherwise, it forces a new calibration sequence.

### ⚠ Challenge 3: Wiring Instability
Breadboard connections for a vibrating mobile robot proved unreliable. Signal noise on the UART lines caused errors, leading to "ghost" inputs or unresponsive motors.
* **Solution:** We transitioned from loose jumper wires to semi-permanent cabling. Critical connections were reinforced with hot glue at the connector bases to provide strain relief. We also implemented a **Checksum (XOR)** validation in the UART protocol to discard corrupted frames before they could affect motor behavior.

---

## 5. Media & Demonstration

* **Early Connections**

<img src="https://github.com/user-attachments/assets/649bde87-b61b-4933-a5ef-0795a282d8ea" width="450" alt="Early Connections">

* **Keypad Control:**

https://github.com/user-attachments/assets/dda9183b-13e5-4316-8b85-7ee06566c8c5

* **Bench Testing:**

https://github.com/user-attachments/assets/29ef4da7-9fe3-4f08-b457-b4d4329dbd00

* **Joystick Control:**

https://github.com/user-attachments/assets/100aa264-1669-4684-886a-45d782bc2c1f

* **Collision Avoidance (Ultrasonic Stop):**

https://github.com/user-attachments/assets/8579bd8f-04f2-405a-9c75-086f21f1bbac

* **In-Place Rotation:**

https://github.com/user-attachments/assets/c568624a-379e-4412-8c64-9421fc660d6f

* **Pre-defined Path Execution:**

https://github.com/user-attachments/assets/a537f1b7-d5e5-4742-91bd-9db7e8c23e17


* **Photos:**
<p float="left">
  <img src="https://github.com/user-attachments/assets/75dac748-2359-4299-8252-a10998dca168" width="400" />
  <img src="https://github.com/user-attachments/assets/27426ff8-f264-4f3b-83b8-bc3c2a219bbf" width="400" />
  <img src="https://github.com/user-attachments/assets/747487dc-e81b-4cb8-98c5-eb1e2ba172ad" width="400" />
  <img src="https://github.com/user-attachments/assets/4ee48d73-b6c2-4b14-b8df-83cd533aa72a" width="400" />
</p>

---

## 6. Future Improvements
* **PCB Integration:** Move from breadboards to a custom PCB to eliminate hot-glue fixes.
* **IMU Stabilization:** Add a gyroscope to the ESP32 for better navigation.
* **Wireless Telemetry:** Utilize the ESP32's Wi-Fi to broadcast robot status to a web dashboard and wireless control.

---

## 7. References
* [STM32L432KC Datasheet](https://www.st.com/resource/en/datasheet/stm32l432kc.pdf)
* [ESP32 Datasheet](https://documentation.espressif.com/esp32_datasheet_en.pdf)
* [Dagu Wild Thumper Manual](https://www.pololu.com/product/1567)
* [Digilent PmodCLP Reference Manual](https://digilent.com/reference/_media/pmod:pmod:pmodCLP_rm.pdf)