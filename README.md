# HyPower Avionics
## Overview
This repository holds the hardware, firmware, and ground support software for our bi-propellant rocket's avionics stack. Every board in the system is designed in-house, from sensing and flight control down to power distribution and RF links between the launch vehicle and the ground station.

## Boards

### Flight Computer (FC)
The core flight avionics package, carrying:
- **Barometer** — altitude and ascent/descent rate
- **IMU** — attitude and acceleration
- **GNSS** — position and velocity
- **LoRa radio** — telemetry downlink to the ground station
### Feed System Control Unit (FSCU)
Drives and monitors the propellant feed system:
- Actuates all valves — **servos and solenoids**
- Reads **pressure transducers** across the feed system
- Runs the closed-loop control logic for our **electronic regulators (EREGS)**
EREGS is a gain scheduling PID-based control loop that steps propellant pressure down from **300 bar to 50 bar** entirely electronically, with no manual mechanical regulator in the loop.
### CAN Gateway
Bridges the rocket's internal CAN bus with the CAN link to the ground, acting as the interface point between the vehicle's avionics network and the ground station. It also acts like a data storage device to access telemetry post launch.
### LoRa Boards (Vehicle + Ground Station)
Matched LoRa radio boards fly on the launch vehicle and sit at the ground station, forming the primary telemetry link. They are using SX1262 at 915Mhz.
### Backplate (Power Distribution + Charging)
The backplate is the electrical backbone of the vehicle avionics — it functions simultaneously as:
- A **Power Distribution Unit (PDU)**, feeding all avionics boards
- A **charging station**, so the stack can be charged without breaking down harnessing
## Harnessing
All inter-board harnessing uses **Harwin** 4-pin and 6-pin connectors for reliable, vibration-rated connections throughout the vehicle.
## Sponsors
None of this flies without the support of our sponsors, who backed this program and made it possible:
- **[Texas Instruments](https://www.ti.com/)** — TI's power circuitry has been rock solid across every board in our stack, from the Flight Computer down to the FSCU.
- **[Harwin](https://www.harwin.com/)** — NASA/ESA-grade connectors used across all of our harnessing, including the 4-pin and 6-pin connectors tying every board together. Despite the space-grade pedigree, they're easy to crimp and solder.
- **[EasyEDA](https://easyeda.com/)** — an easy-to-use ECAD tool that all of our boards are designed in, with simple ordering straight through to **[JLCPCB](https://jlcpcb.com/)** for manufacturing. Can't recommend JLC enough — fast, reliable and consistently good build quality.
- **[RS Components](https://uk.rs-online.com/)** — our go-to for electronic components and sensors.
Thank you for making this program possible. 
## License
