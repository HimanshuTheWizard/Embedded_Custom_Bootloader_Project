# Embedded_Customer_Bootloader_Project
Target Machine : STM32F4XX //
IDE Used	     : STM32CubeMx and Keil //
Purpose		     : Design Custom bootloader for ARM Cortex Mx Machine //
Description	   : 

Application code directory - Bootloader_user_application
Bootloader code directory  - Embedded_Bootloader_Program
Host application directoty - Host

In flash memory, bootloader application has been flashed into the first two sectors of the flash. Base address for the bootloader flash starts from 0x08000000 while the user application base address starts with 0x08008000.

When user programs the controller, bootloader application redirect the execution to reset handler of user application in normal case. user application is a simple demonstration software which shall toggle an yellow LED on board on each user button press.

When user programs the controller, and press the user button before executing the main, the excution is redirected to bootloader application and bootloader program shall execute.

When program enters the bootloader software, you should open the host application and provide the commands from user command prompt to the controller.
pdf file has also been attached to explain various command that the target controller can read and execute.
