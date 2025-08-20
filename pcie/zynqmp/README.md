# Description
This library is supposed to deliver basic functionalities of PCIe fabric, that is used by device driver. Since PCIe device drivers might require complex interrupt handlers, and latency is supposed to be minimized it is supposed to be linked as static library to a process (or a library) that acts as a driver for the device. This static library provides following functionalities:
1. Perfrom setup of the bus.
2. Enumeration of PCIe fabric.
3. Allocation of BARs for each device on the fabric.
4. Provide access to capability structures.
5. Start thread for interrupt handling (both MSI and legacy, MSI-X not supported)

# Using the library
For most device drivers, followin setup should be sufficient:
1. Create PCIe context using the `pci_initContext` - this function should setup bus, enumerate devices and allocate BARs
2. Get needed device using `pci_getDevFromTree` - this function takes PID and VID, and returns pointer to device structure. If such is not found returns NULL.

Having the device you should have access to BDF addressing of the device as well as access to BARs.

# Interrupts
## Legacy
Currentlly there is no mechanism of finding which device signalled interrupt in legacy mode. This would require:
1. Finding which interrupt line was asserted.
2. Getting all devices that have this interrupt line assigned.
3. Running interrupt handler for each device.
## MSI
To register MSI interrupt callback set callback in device structure, and set handler payload. Next use `pci_enableMsiInterrupt`.

# Notes
1. There is `safe_printf` function, you should provide custom implementation - may prove helpful while debugging.
2. There are many PCIe functionalities that have not been implemented, but using `pci_findCapability` you should be able to do what you need.s