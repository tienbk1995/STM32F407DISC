# STM32F407DISC — Embedded Firmware Exercises

**STM32F407 Discovery Board — register-level firmware & peripheral experiments.**  
A personal lab repository for practicing low-level embedded C on the ARM Cortex-M4 (STM32F407).  
This project demonstrates fundamentals that I apply in industrial embedded work (drivers, interrupts, timers, ADC, UART, PWM).

---

## TL;DR
- **Role:** Solo project / learning lab  
- **Platform:**
  STM32F407 Discovery (Cortex-M4, 168 MHz)
  ESP-WROOM-32
- **Language & Tools:** Embedded C, CMSIS, minimal STM32 HAL, GNU Arm Embedded Toolchain / STM32CubeIDE  
- **Focus:** Bare-metal drivers, interrupt design, peripheral integration, system clock configuration

---

## What’s included
- GPIO (LED blink) — register-level + HAL examples  
- Timer interrupts and periodic task scheduling  
- External interrupts (button input) / EXTI configuration  
- UART (polling & interrupt-driven) for debug/console output  
- PWM generation for motor/LED control experiments  
- ADC sampling example (single conversion)  
- Startup code and linker script examples for Cortex-M  
- Makefile / simple build guidance (see **Build & Flash**)
