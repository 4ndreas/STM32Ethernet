Modified library to run on the NUCLEO-H743ZI2 (STM32H743ZIT6U). However, there are some caveats that come with it:

* code is take form the STM32CubeMX generated code version 1.11
* needs modified linker-script found in the ld/ folder
* also the memory has to be initialised via the ```MPU_Config();```
see the ```NUCLEO-H743ZI2``` example for details.

  a detailed description can be found at 
  https://community.st.com/s/article/How-to-create-project-for-STM32H7-with-Ethernet-and-LwIP-stack-working

  section 10 is mostly relevant for this.



# STM32 Ethernet Library for Arduino

With an STM32 board with Ethernet compatibility, this library allows a STM32
board (NUCLEO, DISCOVERY, ...) to connect to the internet.  

This library follows the Ethernet API from Arduino.<br>
For more information about it please visit: http://www.arduino.cc/en/Reference/Ethernet

## Dependency

This library is based on LwIP, a Lightweight TCP/IP stack, available here:

http://git.savannah.gnu.org/cgit/lwip.git

The LwIP has been ported as Arduino library and is available thanks Arduino Library Manager.

Source: https://github.com/stm32duino/LwIP

## Configuration

The LwIP has several user defined options, which is specified from within the `lwipopts.h` file.

This library provides a default user defined options file named `lwipopts_default.h`.

User can provide his own defined options at sketch level by adding his configuration in a file named `STM32lwipopts.h` or
extend the default one by adding some extra configuration in a file named `lwipopts_extra.h`.



## New alternative init procedure **!!!**

There are alternative inits of the Ethernetinterface with following orders:

	Ethernet.begin();
	Ethernet.begin(ip);
	Ethernet.begin(ip, subnet);
	Ethernet.begin(ip, subnet, gateway);
	Ethernet.begin(ip, subnet, gateway, dns);

This is more logical. A MAC address is no more needed and will retrieved internally by the mbed MAC address!

You can get the MAC address with following function, this must done after Ethernet.Begin()
	
	uint8_t *mac;
	Ethernet.begin();
	mac = Ethernet.MACAddress();

You can also set a new user based MAC address, this must done before Ethernet.begin()

	uint8_t newMAC[] = {0x00, 0x80, 0xE1, 0x01, 0x01, 0x01};
	Ethernet.MACAddress(newMAC);
	Ethernet.begin();

## Note

`EthernetClass::maintain()` in no more required to renew IP address from DHCP.<br>
It is done automatically by the LwIP stack in a background task.  

An Idle task is required by the LwIP stack to handle timer and data reception.<br>
This idle task is called inside a timer callback each 1 ms by the
function `stm32_eth_scheduler()`.<br>
A `DEFAULT_ETHERNET_TIMER` is set in the library to `TIM14`.<br>
`DEFAULT_ETHERNET_TIMER` can be redefined in the core variant.<br>
Be careful to not lock the system in a function which disabling IRQ.<br>
Call `Ethernet::schedule()` performs an update of the LwIP stack.<br>

## Wiki

You can find information at https://github.com/stm32duino/wiki/wiki/STM32Ethernet
