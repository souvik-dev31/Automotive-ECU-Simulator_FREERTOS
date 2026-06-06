# Automotive ECU Simulator (FreeRTOS & CAN 2.0A)

A multi-tasking Real-Time Operating System (RTOS) project demonstrating host-based simulation and target-hardware deployment for automotive Electronic Control Units (ECUs). 

This project simulates a vehicle engine management system, executing concurrent sensor data acquisition, processing, and actuator control using **FreeRTOS**. It features standard automotive communication (CAN Bus) and enforces ISO-26262 style functional safety through a high-priority Watchdog supervisor.

## System Architecture

The software architecture is divided into four concurrent FreeRTOS tasks, communicating deterministically via Message Queues and Event Groups:

1. **Sensor Task (Priority 3):** Simulates engine telemetry (Throttle Position % and Engine RPM) and passes data to the core processor via an RTOS Queue.
2. **ECU Core Task (Priority 2):** Unblocks upon receiving sensor data, calculates the required fuel injection timing, and packs the results into a standard CAN 2.0A frame using bitwise shifts.
3. **Actuator Task (Priority 1):** Acts as a CAN node/sniffer, receiving the packed frames, decoding the hexadecimal payload, and triggering the physical hardware (or simulated console output).
4. **Watchdog Supervisor (Priority 4):** A safety-critical task that monitors system health. Every task must "kick the dog" by setting a bit in an Event Group within a strict 2000ms deadline. Failure results in a simulated or physical system reset (via `NVIC_SystemReset()`).

## Key Features
* **FreeRTOS Concurrency:** Safe inter-process communication (IPC) preventing race conditions.
* **CAN Bus Serialization:** Raw 32-bit and 8-bit integers are manually serialized into strict 8-byte hexadecimal payload frames.
* **Functional Safety:** Implementation of a Watchdog timer using FreeRTOS Event Groups to prevent task starvation and infinite loops.
* **Host-to-Target Workflow:** Verified natively on Windows (x86) before being ported to an ARM Cortex-M4 physical microcontroller.

## Repository Structure

The repository reflects the industry standard of testing software logic on a PC before flashing to physical silicon:

* `/Host_Simulator/` - Contains the `main_blinky.c` file configured for the FreeRTOS Windows (MSVC) Simulator. Allows for rapid logic testing and debugging without hardware.
* `/STM32_Target/` - Contains the CubeIDE `main.c` and `.ioc` files ported for the **STM32F446RETx** microcontroller, integrating the RTOS logic with the STM32 Hardware Abstraction Layer (HAL) for physical CAN loopback.

## Getting Started

### Running the Host Simulator (Windows)
1. Download the standard FreeRTOS distribution (v202212.01 or newer).
2. Open `FreeRTOS\Demo\WIN32-MSVC\WIN32.sln` in Visual Studio.
3. Replace the default `main_blinky.c` with the file from the `/Host_Simulator` folder.
4. Compile and run using the Local Windows Debugger.

### Flashing the STM32 Target
1. Open the `.ioc` file in STM32CubeIDE.
2. Ensure `TIM6` is set as the HAL Timebase Source and `USE_NEWLIB_REENTRANT` is enabled.
3. Build the project to generate the `.elf` binary.
4. Flash to the STM32F446RETx via an ST-Link. Connect via UART (115200 Baud) to view the terminal output.

## Sample Terminal Output

```text
--- Automotive ECU Simulator Booting ---

[SENSOR] Reading Throttle: 12% | RPM: 1050
   -> [CAN BUS] ID: 0x1A4 | DLC: 8 | Data: 04 1A 0C 00 78 00 00 00

[SENSOR] Reading Throttle: 14% | RPM: 1100
   -> [CAN BUS] ID: 0x1A4 | DLC: 8 | Data: 04 4C 0E 00 9A 00 00 00

*** [SUPERVISOR] All ECUs Healthy - Watchdog Reset ***

[SENSOR] Reading Throttle: 16% | RPM: 1150
   -> [CAN BUS] ID: 0x1A4 | DLC: 8 | Data: 04 7E 10 00 B0 00 00 00
