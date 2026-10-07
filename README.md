# Marine-System
A Marine SOS sytem node for small outdated vessel to equip them with automated distress signal transmission within a range of ~15km.

<img width="1200" height="1600" alt="WhatsApp Image 2026-08-10 at 12 49 09" src="https://github.com/user-attachments/assets/548396c4-7c2b-48a9-ac08-8b9737986dee" />


## Architecture 
The system consists of three layers:
- Sensing
- State Estimation
- Signal Transmision

  <img width="2720" height="1520" alt="marine_distress_network_architecture_v2" src="https://github.com/user-attachments/assets/2af16268-f37c-4558-a818-b8df36771e4d" />


### Sesning:
- The device uses two sensors to determine sinking: **mpu6050** and **water level detection**
- Uses a fusion algorithm to find sinking of the vessel.
- Uses variable sensing rate going from low to high if it detects sinking, gradual increase in water level, or rapid fluctuation in vessel's orientation.
- Sinking is a slow process hence long time high rate sensing is required when doubted sinking in order to reduce false postiives.
- A gps sensor is also present to send live location of the distressed vessel in case of sinking.
- The distress signal transmission is achieved using LoRa protocol that uses **E22 900T30D** configured to work at 865 MHz frequency.

### State Estimation
- The MCU **stm32 blue pill** uses the sensor input to determine the state of the vessel.
- It utilises variable sampling of sensors to verify the sinking state if doubted.
