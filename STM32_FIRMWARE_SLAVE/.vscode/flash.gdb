set pagination off
set confirm off
set verbose off
set remotetimeout 20

file ./build/Debug/STM32_FIRMWARE_SLAVE.elf

target extended-remote \\.\COM6

monitor swdp_scan
attach 1
monitor halt
monitor erase_mass
load
monitor reset
detach
quit