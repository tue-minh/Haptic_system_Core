set pagination off

target extended-remote \\.\COM6

monitor swdp_scan
attach 1
monitor halt
monitor erase_mass

file ./build/Debug/MM_control.elf
load

monitor reset
detach
quit