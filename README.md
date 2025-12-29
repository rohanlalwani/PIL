# Grid-Connected Three-Phase Inverter Control (C2000 F28379D)

## Project Overview
This project implements a digital control system for a **10 kVA Three-Phase Grid-Connected Inverter (GCI)** using the **TI C2000 F28379D LaunchPad**. The system is designed to inject 3 kW of active power and 3 kVAR of reactive power into a 415V, 50Hz grid.

The implementation utilizes a **Synchronous Reference Frame (SRF) PLL** for grid synchronization and a **dq-frame current controller** to regulate power injection. The control algorithms are discretized using the **Tustin (Bilinear) transformation** and executed in a Processor-in-the-Loop (PIL) configuration.

## Key Features
* **SRF-PLL:** Grid synchronization using a PI-based Phase Locked Loop in the dq-frame.
* **Current Control:** Decoupled inner current control loops ($i_d$, $i_q$) for precise regulation.
* **Power Control:** Outer control loops for Active (P) and Reactive (Q) power tracking.
* **Plant Modeling:** Discrete-time simulation of the L-filter and grid dynamics running directly on the MCU for PIL testing.

## Documentation & Code
* **Full Technical Report:** [View PDF](./docs/GCI_Controller_Report.pdf) - Detailed IEEE-style report covering the mathematical modeling, control loop design, and PIL results.
* **Source Code:** [View Source](./src/main.c) - Complete C implementation for the TMS320F28379D MCU.

## Technical Specifications
| Parameter | Value |
| :--- | :--- |
| **Grid Voltage** | 415V (Line-Line), 50 Hz |
| **DC Bus Voltage** | 800 V |
| **Switching Frequency** | 10 kHz |
| **Controller** | PI (Tustin Discretized) |

## How to Run
1.  **Hardware Setup:** Connect the TI F28379D LaunchPad to your PC.
2.  **Software:** Import the `src` files into Code Composer Studio (CCS).
3.  **PIL Simulation:** The code is set to PIL Mode. Use the CCS Expression Window to change `pref` (Active Power) or `qref` (Reactive Power) in real-time.

## Author
**Rohan** - *Electrical Engineering*