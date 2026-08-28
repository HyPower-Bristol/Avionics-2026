# HYPOWER Avionics


## Overview

This repository holds the hardware, firmware, and ground support software for our bi-propellant rocket's avionics stack. Every board in the system is designed in-house, from sensing and flight control down to power distribution and RF links between the launch vehicle and the ground station.

## Boards

### Flight Computer (FC)

The core flight avionics package, carrying:
- **Barometer** — altitude and ascent/descent rate
- **IMU** — attitude and acceleration
- **GNSS** — position and velocity
- **LoRa radio** — telemetry downlink to the ground station
The Flight Computer has been flight tested and performed flawlessly. LoRa telemetry and GNSS lock were both maintained and verified out to **6 km**.

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

None of this flies without the support of our sponsors:

- **[Texas Instruments](https://www.ti.com/)** — for the semiconductor components at the heart of our sensing, power, and control boards.
- **[Harwin](https://www.harwin.com/)** — for the connectors used across all of our harnessing, including the 4-pin and 6-pin connectors tying every board together.
- **[EasyEDA](https://easyeda.com/)** — for the PCB design tooling used to lay out our boards, paired with **[JLCPCB](https://jlcpcb.com/)** for manufacturing.

Thank you for making this program possible.
## License

_Add your license here._
