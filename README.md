# STM32F407 Discovery Board – Embedded Systems Practice

This repository contains my embedded firmware development exercises and experiments on the **STM32F407 Discovery Board**, focusing on **bare-metal C programming, peripheral control, and system-level understanding** of ARM Cortex-M4 microcontrollers.

---

## Overview

The goal of this project is to explore **low-level driver development** and **embedded software design** on STM32F4 MCU — from register-level initialization to peripheral integration — as a foundation for advanced automotive and real-time systems.

This serves as a personal knowledge to:
- Strengthen my embedded C skills and MCU peripheral handling
- Explore interrupt-driven design and memory-mapped I/O
- Practice structured firmware organization for scalability
- Prepare for higher-level frameworks such as **AUTOSAR MCAL** and **RTOS integration**

---

## Hardware & Tools

| Component | Description |
|------------|-------------|
| **Board** | STM34F407DISC KIT | ESP32 Devkit V1 |
| **IDE/Toolchain** | STM32CubeIDE | Arduino IDE 2.3.6 |
| **Debugger** | ST-Link V2 |
| **Language** | Embedded C |
| **Libraries** | EPS32 Dev Module | 
| **Build System** | IDE project file |
| **OS** | Windows / Linux |

---

## Features Implemented

- GPIO configuration and LED blinking (register-level + HAL)
- Timer interrupt setup for periodic tasks
- External interrupt (EXTI) with button input
- USART serial communication (polling + interrupt)
- PWM signal generation
- ADC input sampling
- NVIC and interrupt vector table configuration
- System clock setup (PLL, HSE, SysTick)
- SPI Communication
