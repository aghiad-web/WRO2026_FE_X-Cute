# WRO 2026 — Future Engineers | Team X-CUTE

Engineering documentation for the self-driving vehicle built by **Team X-CUTE** for the World Robot Olympiad 2026 Future Engineers category.

---

## Content

| Folder | Description |
|---|---|
| `t-photos` | Team photos — one official group photo and one informal/fun photo |
| `v-photos` | Vehicle photos from all six angles (front, back, left, right, top, bottom) |
| `video` | Link to the driving demonstration videos |
| `schemes` | Electromechanical schematics — wiring diagrams, PCB schematic and layout |
| `src` | All source code — Raspberry Pi vision/control scripts and STM32 firmware |
| `models` | 3D-printable models (STL) and SolidWorks source files for custom parts |
| `other` | Supporting files — datasheets, cost report, assembly notes |

---

## 1. Introduction

Our robot bridges advanced computer vision for strategic decision-making with real-time motion control. The vehicle uses an **Ackermann steering mechanism** and a hybrid sensory system that combines Time-of-Flight laser sensors for high-accuracy distance measurement with ultrasonic sensors acting as a fast-response safety and wall-following layer.

High-level visual perception runs on a **Raspberry Pi 5**, while an **STM32F411 microcontroller** executes real-time PID control. This separation ensures seamless navigation, dynamic obstacle dodging, and precise final parking.

---

## 2. The Team

**Team X-CUTE**

| Role | Name |
|---|---|
| Coach | Aghiad Jomah |
| Member | Mohamad Alzoubi |
| Member | Elias Kuzma |

Our team integrates embedded systems engineering with computer vision to build self-driving vehicles capable of analyzing their environment and making independent, reliable navigational decisions in dynamic conditions.

> Team photos are in [`t-photos`](t-photos).

---

## 3. Mobility Management

### 3.1 Chassis Design

The chassis was precision-designed in **SolidWorks** and fully 3D-printed, divided into three functional layers to balance weight distribution, mechanical isolation, and cable routing:

| Layer | Contents | Purpose |
|---|---|---|
| **Top (3rd)** | Custom PCB, STM32F411, camera module | Elevating the camera gives a clear, wide field of view for accurate track and pillar detection |
| **Middle (2nd)** | Raspberry Pi 5, motor driver | High-level processing and decision-making hub |
| **Bottom (1st)** | Drive motor, steering servo, battery | Places the heaviest components lowest for a low centre of gravity and stability during sharp cornering |

### 3.2 Custom 3D-Printed Parts

All source models are available in [`models`](models).

- **Three main chassis layers** — custom lightweight structure with pre-designed mounting holes, integrated standoffs, and optimised cable routing paths.
- **Ackermann steering knuckles** — designed to achieve the exact Ackermann geometry for our wheelbase, with optimised pivot points for maximum steering angle without wheel scrub.
- **DC motor mount** — secures the Ga25 motor and maintains alignment between the motor shaft and differential gear to prevent power loss.
- **Ultrasonic sensor holders** — set the mounting height and angle to detect track walls reliably while avoiding false floor reflections.
- **Laser sensor mounts** — hold the VL53L0X modules at a fixed angle for accurate pillar targeting.

### 3.3 Steering — Ackermann Geometry

Ackermann geometry allows the inner wheel to steer at a sharper angle than the outer wheel, minimising tire slip and scrub while maintaining an optimal centre of rotation.

Where `w` is the track width, `l` the wheelbase, `φᵢ` the inner wheel angle, `φₒ` the outer wheel angle, and `r` the distance from the instantaneous centre of curvature (ICC) to the vehicle centre:

```
tan(φ)  = l / r
tan(φᵢ) = l / (r − w/2)
tan(φₒ) = l / (r + w/2)

φᵢ = arctan( 2l·sin(φ) / (2l·cos(φ) − w·sin(φ)) )
φₒ = arctan( 2l·sin(φ) / (2l·cos(φ) + w·sin(φ)) )
```

Actuation uses a **DYNAMIXEL XL330 smart servo**. Unlike standard RC servos driven by PWM, it communicates over a half-duplex serial bus, letting the STM32 send precise angular commands and read back the exact motor position.

### 3.4 Drivetrain

| Component | Specification |
|---|---|
| **Drive motor** | Ga25 brushed DC gear motor — 12 V, ≈600 RPM, ≈10:1 gearbox, ≈0.4 kg·cm continuous torque, ≈1.2 A stall |
| **Steering servo** | DYNAMIXEL XL330 — 5 V, ≈5.3 kg·cm stall torque, UART/TTL half-duplex, internal PID with position/velocity/load/temperature feedback |
| **Differential** | Gear differential on the rear axle — allows the outer wheel to rotate faster while cornering, preventing skid |
| **Wheels** | Silicone — high grip and traction, minimising slippage under acceleration and braking |

---

## 4. Power and Sense Management

### 4.1 Sensors

| Sensor | Qty | Role |
|---|---|---|
| **VL53L0X** ToF laser | 2 | Primary sensor for precise dynamic pillar dodging — mm-level accuracy, up to 200 cm, 50 Hz, immune to ambient light |
| **US-100** ultrasonic | 3 | Wall-following and safety net — 2–450 cm, built-in temperature compensation for stable readings regardless of motor or ambient heat |
| **BNO085** 9-DoF IMU | 1 | Heading estimation via on-chip sensor fusion — ≈1.5° static / ≈3.5° dynamic accuracy, resistant to electromagnetic interference |
| **FANTECH LUMINOUS C30** camera | 1 | USB webcam, 2K @ 30 FPS, ≈106° ultra-wide FOV — line and pillar detection via OpenCV |

### 4.2 Drivers

- **IBT-4 DC Motor Driver** — high-current MOSFET H-bridge, 5–15 V, up to 50 A peak. Provides large headroom over the Ga25's 1.2 A stall current and protects against back-EMF during active braking. Interfaces directly with 3.3 V STM32 logic, no level shifters required.
- **Waveshare Serial Bus Servo Driver** — converts standard full-duplex UART to the half-duplex single-wire bus the XL330 requires, with hardware-level automatic TX/RX direction switching. Routes servo power separately, isolating motor current from logic circuits.

### 4.3 Controllers

Our system uses a **Hybrid Control Architecture** that separates high-level processing from low-level real-time motion control.

**Raspberry Pi 5 (8 GB)** — the brain. Runs Python control scripts and OpenCV computer vision, processes camera frames, applies the Y-axis depth-sorting algorithm for red and green pillars, and sends strategic commands to the STM32 over UART. The 2.4 GHz quad-core Cortex-A76 and 8 GB LPDDR4X allow 2K-resolution processing at high frame rates without bottlenecks.

**STM32F411 (BlackPill)** — the muscle. Receives strategic commands over UART and translates them into precise PWM motor signals, handles ultrasonic hardware interrupts, and reads the IMU. Linux is not designed for hard real-time tasks; the STM32 provides **deterministic execution**, ensuring PID calculations, motor updates, and sensor reads occur with microsecond precision and no jitter. Its hardware FPU accelerates the floating-point math the control loops require.

### 4.4 Power Management

The electrical system follows a **"Separation of Power Domains"** philosophy:

- **Logic power** — the Raspberry Pi and its camera run from a dedicated 20 W power bank. This complete isolation prevents OS brownouts caused by sudden current draw from the motors.
- **Actuator power** — a 3S Li-Po (11.1 V, 2200 mAh, 30C) supplies the lower layer. The high discharge rate delivers burst current for the motor and servo during acceleration and sudden stops without voltage sag.
- **Voltage regulation** — an XL4015 DC-DC buck converter steps 11.1 V down to a clean, regulated 5 V for the STM32 and sensors, at up to 96% efficiency with thermal shutdown and short-circuit protection.

<details>
<summary><b>Full current budget</b></summary>

| Component | Qty | Supply | Idle | Active | Peak/Stall |
|---|---|---|---|---|---|
| Raspberry Pi 5 (8 GB) | 1 | 5.0 V | ~600–800 mA | ~1.5–2.0 A | up to 5.0 A |
| FANTECH C30 camera | 1 | 5 V (Pi USB) | — | ~150–250 mA | ~300 mA |
| STM32F411 | 1 | 3.3 / 5 V | ~10 mA | ~25–35 mA | ~50 mA |
| BNO085 IMU | 1 | 3.3 V | ~3 mA | ~10–15 mA | — |
| US-100 ultrasonic | 3 | 5.0 V | < 2 mA | ~3–5 mA | — |
| VL53L0X laser | 2 | 3.3–5.0 V | < 5 µA | ~20 mA | ~40 mA |
| DYNAMIXEL XL330 | 1 | 5.0 V | ~15 mA | ~150–300 mA | ~1.5 A |
| Ga25 DC motor | 1 | 12 V | — | ~100–400 mA | ~1.2 A |
| IBT-4 driver | 1 | 5 V logic / 12 V load | ~2 mA | driver losses | 50 A peak rating |
| Waveshare servo driver | 1 | 5.0 V | ~2 mA | servo load pass-through | — |
| XL4015 converter | 1 | 12 V in / 5 V out | ~5 mA | load dependent | 5 A peak rating |

</details>

### 4.5 Custom PCB

To eliminate the unreliability of loose jumper wires and breadboards, we designed a custom PCB that acts as the central nervous system of the hardware — routing power and data between the STM32, sensor arrays, motor drivers, and the XL4015 power module.

The schematic enforces complete isolation between logic circuits and high-current actuators, with properly sized power traces, pull-up resistors on the I²C lines, and organised header pins. The layout accounts for EMI and thermal dissipation: the XL4015 and BlackPill are placed to minimise signal travel distance, and extensive copper ground pours provide a common reference and heat spreading.

> Schematics and layout files are in [`schemes`](schemes).

---

## 5. Obstacle Management

### 5.1 System Architecture

Rather than heavy middleware, we use a lightweight **Master–Slave architecture** over direct UART:

- **Master node** — Raspberry Pi 5 running Python. Captures frames, processes them via OpenCV, and issues strategic commands (`TURN_LEFT`, `DODGE_RIGHT`, …).
- **Execution node** — STM32F411. Receives short string commands and folds them into a **non-blocking Finite State Machine** that updates motor speeds and PID offsets without halting distance or IMU sensor reads.

### 5.2 Open Challenge — "Direction Lock"

The goal is three laps without wall contact in the shortest time. We run a wall-following algorithm on the ultrasonic sensors, driving the error between right and left distances toward zero with a PID controller.

For corners, the camera does not re-evaluate colour at every turn. On start, it monitors the floor: an **orange line first locks the direction as Clockwise**; a **blue line first locks Counter-Clockwise**. For the remaining 11 corners the robot turns according to the locked direction as soon as any line colour is seen — eliminating visual confusion and guaranteeing track stability.

### 5.3 Image Processing

Frames from the tilt-down camera are converted from RGB to **HSV** to resist colour fluctuation under changing light. We apply masks isolating the four relevant colours (blue, orange, red, green), run morphological filters to remove noise, and extract bounding boxes to determine each element's position and size in frame.

### 5.4 Obstacle Challenge — Y-Axis Depth Sorting

Pillars require higher spatial accuracy than corners. To handle complex scenes — for example red and green pillars in the same frame — we developed the **Y-Axis Depth Sorting** algorithm:

1. Collect the coordinates of all detected pillars and sort them in descending order by the lowest point of their bounding box (`bottom_y`).
2. Ignore all distant pillars; interact only with the pillar of highest `Y` value — the one nearest the front of the robot.
3. If the nearest pillar is **red**, the Pi sends `DODGE_RIGHT`; if **green**, `DODGE_LEFT`. The STM32 translates the command into a dynamic `Lane_Offset` bias inside the distance-sensor PID equation, smoothly hugging the corresponding wall.

Because the dodge is expressed as a PID bias rather than a discrete manoeuvre, the robot performs continuous slalom movement without ever needing to stop.

---

## 6. Performance Videos

| Challenge | Video |
|---|---|
| Open Challenge | *[link]* |
| Obstacle Challenge | *[link]* |

---

## 7. Problems We Encountered

| Problem | Solution |
|---|---|
| **Camera field of view** — the standard Pi camera's narrow FOV made it hard to see the full track width or spot peripheral pillars in time. | Replaced with the FANTECH C30 wide-angle webcam, letting the vision pipeline capture surrounding detail and recognise obstacles from a safe distance. |
| **Ackermann steering angle limits** — the initial mechanism capped our maximum steering angle, widening the turning radius and pushing the robot toward outer walls. | Redesigned the knuckles and tie rods in SolidWorks with optimised pivot points and lever lengths, significantly increasing front-wheel range of motion while preserving correct Ackermann intersection. |
| **Frame flickering** — room lighting and fluorescent lamp frequency destabilised our colour masks. | Disabled auto white balance and set shutter speed and exposure programmatically via OpenCV to match ambient lighting frequency. |
| **Processor data-flow desync** — continuous UART streaming from the Pi overflowed the STM32 buffer, delaying motor response. | Replaced streaming with a lightweight event-based protocol that sends short commands only on state change, plus a non-blocking FSM on the STM32 so serial reads never halt other operations. |
| **Merging pillar contours** — closely spaced pillars merged into one contour from the low camera perspective, misread as a single obstacle. | Raised the camera to the third layer and optimised its tilt-down angle, restoring visual separation and letting depth sorting work accurately. |

---

## 8. Improvements and Future Modifications

**Mobility**
- **Aluminium CNC upgrade** — replace the printed steering knuckles and motor mounts with CNC-machined aluminium to eliminate joint flex at speed and improve steering precision.
- **Differential upgrade** — move to a sealed metal differential with lubricating fluid to cut internal friction and improve torque transfer efficiency.

**Power and Sensing**
- **Battery Management System** — integrate a BMS module into the PCB to monitor individual cell voltages and real-time current draw, reported over UART so the system can dynamically cap top speed and prevent voltage drops.
- **LiDAR integration** — a 2D LiDAR (e.g. RPLIDAR) would give a 360° point cloud, reducing discrete sensor count and improving localisation accuracy within the track.

**Obstacle Management**
- **Deep learning for vision** — replace HSV colour masking with a lightweight model such as YOLO, making detection immune to drastic lighting changes and colour distortion.
- **Path caching** — pillar placement is fixed across all three laps under WRO rules. Memorising pillar colours and coordinates during a slower first lap would let laps two and three run at maximum speed without re-evaluating every frame.

---

## 9. Conclusion

Our goal was never simply to build a robot that completes the track, but to engineer a system where every component was chosen for a defensible reason and every subsystem validated through testing rather than assumption.

The defining decision was the **Hybrid Control Architecture** — separating high-level perception on the Raspberry Pi from hard real-time execution on the STM32. This let each processor do what it does best: computer vision without timing pressure, and deterministic PID control without operating-system jitter.

Equally important was iterative refinement. Nearly every subsystem here exists in its current form because an earlier version failed under test — the steering knuckles were redesigned after we hit our geometry's angular limits, the camera moved up a layer after merged contours revealed the cost of a low perspective, and the communication protocol was rewritten after buffer overflows appeared. Each failure taught us more than a first-attempt success would have.

Beyond the technical outcome, this project developed our capabilities in mechanical design, PCB layout, embedded firmware, computer vision, and the systems thinking required to make these disciplines work together.

---

*Team X-CUTE — WRO 2026 Future Engineers*