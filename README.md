# Marine-System
A Marine SOS node for small, older vessels. It watches for signs of sinking and, when it detects them (or when the crew presses the SOS button), broadcasts an automated distress signal over LoRa. Nearby vessels running the same firmware can acknowledge it and be assigned rescue roles. The system is designed for a range of ~15 km (design target, not yet field-validated).

<img width="1200" height="1600" alt="WhatsApp Image 2026-08-10 at 12 49 09" src="https://github.com/user-attachments/assets/548396c4-7c2b-48a9-ac08-8b9737986dee" />

## Key Features
- Automatic sinking detection from two sensors: MPU6050 tilt and an analog water level sensor.
- Variable sensing rate: slow polling when idle, faster polling once either sensor crosses an early-warning threshold.
- Manual SOS button on a hardware interrupt that overrides all sensor logic.
- Live GPS position (u-blox NEO-M8L) embedded in every distress packet.
- Long-range LoRa link using an Ebyte E22-900T30D (SX1262, 30 dBm) in transparent mode.
- Custom compact binary protocol with four packet types: DISTRESS, ACK, DISPATCH and RELAY.
- Same firmware on every vessel. A node's role (distress, helper, relayer, idle) is decided at runtime, not at compile time.
- Local alarm (buzzer and LED) while a distress is active.


## Architecture 
The system consists of three layers:
- Sensing
- State Estimation
- Signal Transmision

  <img width="2720" height="1520" alt="marine_distress_network_architecture_v2" src="https://github.com/user-attachments/assets/2af16268-f37c-4558-a818-b8df36771e4d" />


### Sesning:
<img width="606" height="299" alt="image" src="https://github.com/user-attachments/assets/ce1f031b-2e9f-4bc6-b635-1cd522289fbf" />

- The device uses two sensors to determine sinking: **mpu6050** and **water level detection**
- Uses a fusion algorithm to find sinking of the vessel.
- Uses variable sensing rate going from low to high if it detects sinking, gradual increase in water level, or rapid fluctuation in vessel's orientation.
- Sinking is a slow process hence long time high rate sensing is required when doubted sinking in order to reduce false postiives.
- A gps sensor is also present to send live location of the distressed vessel in case of sinking.
- The distress signal transmission is achieved using LoRa protocol that uses **E22 900T30D** configured to work at 865 MHz frequency.

### State Estimation

<img width="1228" height="519" alt="image" src="https://github.com/user-attachments/assets/ca916ea5-0618-4656-ae56-12a8a30cec00" />

- The MCU **stm32 blue pill** uses the sensor input to determine the state of the vessel.
- It utilises variable sampling of sensors to verify the sinking state if doubted.
- The tilt angles are based on the heel angles used in IMO righting-lever (GZ curve) stability criteria: 30° and 40° are standard checkpoints, and 40° (or the downflooding angle, if smaller) is the upper limit of the area criteria. Those criteria are written for ships, so treat 15° and 40° here as generic defaults. A real vessel's downflooding angle depends on its design.
- The water thresholds are uncalibrated placeholders. They must be calibrated for your specific sensor by dipping it to known depths and logging raw analogRead() values.
- TILT_WARNING_DEG and WATER_WARNING_RAW are defined for a future middle tier but are not used yet.

### Signal Transmission
<img width="945" height="440" alt="image" src="https://github.com/user-attachments/assets/c5ef4883-710c-4ded-9db9-a93d3f0e2da9" />

- **Distress**: A vessel in distress broadcasts a DISTRESS packet and opens a 10 s ACK window.
ACK. Every vessel that hears it replies with an ACK containing its own position. To limit collisions, replies are delayed by a jitter proportional to distance from the distressed vessel (nearer vessels answer first) plus 0 to 500 ms of randomness.
- **Dispatch**: When the window closes, the distressed vessel computes distances from the raw coordinates and assigns:
- **Helper**: the responder closest to the distressed vessel.
- **Relayer**: the responder closest to a reference port coordinate (PORT_LAT_DEG, PORT_LON_DEG).
- If one vessel is closest on both counts, it receives a single DISPATCH with both role bits set.
- **Relay**: RELAY packets are forwarded hop by hop with a hop counter and a path list for loop detection. The chain is dropped at MAX_HOPS = 10.
- **De-duplication**: A 32-entry seen-message cache (10 min TTL) prevents repeated handling of the same message.
