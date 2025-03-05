warning: LF will be replaced by CRLF in stm32f407GDisc_drivers/Src/togglebutton_main.c.
The file will have its original line endings in your working directory
[1mdiff --git a/stm32f407GDisc_drivers/.cproject b/stm32f407GDisc_drivers/.cproject[m
[1mindex 1f521d6..177cf32 100644[m
[1m--- a/stm32f407GDisc_drivers/.cproject[m
[1m+++ b/stm32f407GDisc_drivers/.cproject[m
[36m@@ -71,12 +71,12 @@[m
 							<tool id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.objcopy.symbolsrec.1369562199" name="MCU Output Converter Motorola S-rec with symbols" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.objcopy.symbolsrec"/>[m
 						</toolChain>[m
 					</folderInfo>[m
[31m-					<fileInfo id="com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug.1282819938.583852677" name="main.c" rcbsApplicability="disable" resourcePath="Src/main.c" toolsToInvoke="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001.1775487470">[m
[31m-						<tool id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001.1775487470" name="MCU/MPU GCC Compiler" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001"/>[m
[32m+[m					[32m<fileInfo id="com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug.1282819938.1340420202" name="togglebutton_main.c" rcbsApplicability="disable" resourcePath="Src/togglebutton_main.c" toolsToInvoke="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001.497793763">[m
[32m+[m						[32m<tool id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001.497793763" name="MCU/MPU GCC Compiler" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.1862130001"/>[m
 					</fileInfo>[m
 					<sourceEntries>[m
 						<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Inc"/>[m
[31m-						<entry excluding="main.c" flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Src"/>[m
[32m+[m						[32m<entry excluding="togglebutton_main.c|main.c" flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Src"/>[m
 						<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Startup"/>[m
 						<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="drivers"/>[m
 					</sourceEntries>[m
[1mdiff --git a/stm32f407GDisc_drivers/Src/togglebutton_main.c b/stm32f407GDisc_drivers/Src/togglebutton_main.c[m
[1mindex 43bfd46..ab5de01 100644[m
[1m--- a/stm32f407GDisc_drivers/Src/togglebutton_main.c[m
[1m+++ b/stm32f407GDisc_drivers/Src/togglebutton_main.c[m
[36m@@ -28,6 +28,11 @@[m [mvoid SoftwareDelay(uint32_t tick)[m
 	for (uint32_t i = 0; i <= tick; i++);[m
 }[m
 [m
[32m+[m[32mvoid EXTI0_IRQHandler(void)[m
[32m+[m[32m{[m
[32m+[m	[32mGPIO_IRQHandling(0);[m
[32m+[m[32m}[m
[32m+[m
 int main(void)[m
 {[m
 	GPIO_Handle_t button;[m
[1mdiff --git a/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx.h b/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx.h[m
[1mindex 6b7885a..11ad1b1 100644[m
[1m--- a/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx.h[m
[1m+++ b/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx.h[m
[36m@@ -11,6 +11,28 @@[m
 #include <stdint.h>[m
 [m
 #define __vo volatile[m
[32m+[m[32m/************************************************ CPU registers base address ************************************************/[m
[32m+[m[32m// NVIC_ISER -> Enable interrupt[m
[32m+[m[32m#define NVIC_ISER0						((__vo uint32_t *)0xE000E100)[m
[32m+[m[32m#define NVIC_ISER1						((__vo uint32_t *)0xE000E104)[m
[32m+[m[32m#define NVIC_ISER2						((__vo uint32_t *)0xE000E108)[m
[32m+[m[32m#define NVIC_ISER3						((__vo uint32_t *)0xE000E10C)[m
[32m+[m[32m// NVIC_ICER -> Disable interrupt[m
[32m+[m[32m#define NVIC_ICER0						((__vo uint32_t *)0xE000E180)[m
[32m+[m[32m#define NVIC_ICER1						((__vo uint32_t *)0xE000E184)[m
[32m+[m[32m#define NVIC_ICER2						((__vo uint32_t *)0xE000E188)[m
[32m+[m[32m#define NVIC_ICER3						((__vo uint32_t *)0xE000E18C)[m
[32m+[m
[32m+[m[32m/*[m
[32m+[m[32m * ARM Cortex Mx Processor Priority Register Address Calculation[m
[32m+[m[32m */[m
[32m+[m[32m#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)[m
[32m+[m
[32m+[m[32m/*[m
[32m+[m[32m * ARM Cortex Mx Processor number of priority bits implemented in Priority Register[m
[32m+[m[32m */[m
[32m+[m[32m#define NO_PR_BITS_IMPLEMENTED  4[m
[32m+[m
 [m
 /************************************************ Peripheral registers base address ************************************************/[m
 [m
[36m@@ -270,6 +292,37 @@[m [mtypedef struct[m
 #define GPIOH_REG_RESET()				do {RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);} while(0)[m
 #define GPIOI_REG_RESET()				do {RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8);} while(0)[m
 [m
[32m+[m[32m/*[m
[32m+[m[32m * IRQ(Interrupt Request) Numbers of STM32F407x MCU[m
[32m+[m[32m * NOTE: update these macros with valid values according to your MCU[m
[32m+[m[32m * TODO: You may complete this list for other peripherals[m
[32m+[m[32m */[m
[32m+[m
[32m+[m[32m#define IRQ_NO_EXTI0 		6[m
[32m+[m[32m#define IRQ_NO_EXTI1 		7[m
[32m+[m[32m#define IRQ_NO_EXTI2 		8[m
[32m+[m[32m#define IRQ_NO_EXTI3 		9[m
[32m+[m[32m#define IRQ_NO_EXTI4 		10[m
[32m+[m[32m#define IRQ_NO_EXTI9_5 		23[m
[32m+[m[32m#define IRQ_NO_EXTI15_10 	40[m
[32m+[m[32m#define IRQ_NO_SPI1			35[m
[32m+[m[32m#define IRQ_NO_SPI2         36[m
[32m+[m[32m#define IRQ_NO_SPI3         51[m
[32m+[m[32m#define IRQ_NO_SPI4[m
[32m+[m[32m#define IRQ_NO_I2C1_EV     	31[m
[32m+[m[32m#define IRQ_NO_I2C1_ER      32[m
[32m+[m[32m#define IRQ_NO_USART1	    37[m
[32m+[m[32m#define IRQ_NO_USART2	    38[m
[32m+[m[32m#define IRQ_NO_USART3	    39[m
[32m+[m[32m#define IRQ_NO_UART4	    52[m
[32m+[m[32m#define IRQ_NO_UART5	    53[m
[32m+[m[32m#define IRQ_NO_USART6	    71[m
[32m+[m
[32m+[m[32m// IRQ Priority[m
[32m+[m[32m#define NVIC_IRQ_PRI0   	 0[m
[32m+[m[32m#define NVIC_IRQ_PRI15   	 15[m
[32m+[m
[32m+[m
 /* Generic macro */[m
 #define ENABLE 1[m
 #define DISABLE 0[m
[1mdiff --git a/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx_gpio_driver.h b/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx_gpio_driver.h[m
[1mindex 2436242..543f9ad 100644[m
[1m--- a/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx_gpio_driver.h[m
[1m+++ b/stm32f407GDisc_drivers/drivers/Inc/stm32f407xx_gpio_driver.h[m
[36m@@ -129,8 +129,8 @@[m [mvoid GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);[m
 /*[m
  * IRQ Configuration and ISR handling[m
  */[m
[31m-void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t EnorDi);[m
[31m-void GPIO_IRQPriorityConfig(void);[m
[32m+[m[32mvoid GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);[m
[32m+[m[32mvoid GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority);[m
 void GPIO_IRQHandling(uint8_t IRQNumber);[m
 [m
 /*[m
[1mdiff --git a/stm32f407GDisc_drivers/drivers/Src/stm32f407xx_gpio_driver.c b/stm32f407GDisc_drivers/drivers/Src/stm32f407xx_gpio_driver.c[m
[1mindex 51e0ed0..33285d5 100644[m
[1m--- a/stm32f407GDisc_drivers/drivers/Src/stm32f407xx_gpio_driver.c[m
[1m+++ b/stm32f407GDisc_drivers/drivers/Src/stm32f407xx_gpio_driver.c[m
[36m@@ -379,17 +379,67 @@[m [mvoid GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)[m
 /*[m
  * IRQ Configuration and ISR handling[m
  */[m
[31m-void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t EnorDi)[m
[32m+[m[32mvoid GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)[m
 {[m
[32m+[m	[32mif (ENABLE == EnorDi)[m
[32m+[m	[32m{[m
[32m+[m		[32mif (IRQNumber <= 31)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ISER0 |= (1 << IRQNumber);[m
[32m+[m		[32m}[m
[32m+[m		[32melse if (IRQNumber > 31 && IRQNumber < 64)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ISER1 |= (1 << (IRQNumber % 32));[m
[32m+[m		[32m}[m
[32m+[m		[32melse if (IRQNumber >= 64 && IRQNumber < 96)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ISER2 |= (1 << (IRQNumber % 64));[m
[32m+[m		[32m}[m
[32m+[m		[32melse[m
[32m+[m		[32m{[m
[32m+[m
[32m+[m		[32m}[m
[32m+[m	[32m}[m
[32m+[m	[32melse // Disable[m
[32m+[m	[32m{[m
[32m+[m		[32mif (IRQNumber <= 31)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ICER0 |= (1 << IRQNumber);[m
[32m+[m		[32m}[m
[32m+[m		[32melse if (IRQNumber > 31 && IRQNumber < 64)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ICER1 |= (1 << (IRQNumber % 32));[m
[32m+[m		[32m}[m
[32m+[m		[32melse if (IRQNumber >= 64 && IRQNumber < 96)[m
[32m+[m		[32m{[m
[32m+[m			[32m*NVIC_ICER2 |= (1 << (IRQNumber % 64));[m
[32m+[m		[32m}[m
[32m+[m		[32melse[m
[32m+[m		[32m{[m
[32m+[m
[32m+[m		[32m}[m
[32m+[m	[32m}[m
 [m
 }[m
[31m-void GPIO_IRQPriorityConfig(void)[m
[32m+[m
[32m+[m[32mvoid GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t Priority)[m
 {[m
[32m+[m	[32m//1. first lets find out the ipr register[m
[32m+[m	[32muint8_t iprx = IRQNumber / 4;[m
[32m+[m	[32muint8_t iprx_section  = IRQNumber %4 ;[m
 [m
[32m+[m	[32muint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;[m
[32m+[m
[32m+[m	[32m*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( Priority << shift_amount );[m
 }[m
[32m+[m
 void GPIO_IRQHandling(uint8_t IRQNumber)[m
 {[m
[31m-[m
[32m+[m	[32m// clear the EXTI->PR pending int[m
[32m+[m	[32mif (EXTI->PR & (1 << IRQNumber))[m
[32m+[m	[32m{[m
[32m+[m		[32mEXTI->PR |= (1 << IRQNumber);[m
[32m+[m	[32m}[m
 }[m
 [m
 /*[m
