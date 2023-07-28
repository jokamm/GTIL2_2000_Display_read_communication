# GTIL2_2000_Display_read_communication

This SW is for Raspberry Pi Pico. It will be reading data from the LCD display communication of the
GTIL2 2000 (or 1000) and the RS232 communication. The data is buffered in the Pico, so it can be 
send on request. The request comes from a Control-PC (I use a Raspberry Pi 4), using a ModBus 
protocol that is compatible with the ModBus protocol of the DALY BMS.

It uses the 2 UART ports of the Pico and also listens to the serial data line going to the LCD display 
(Rx is on the LCD Display side). The serial data line is connected to a Input GPIO Port of the Pico
making it a 3rd UART Rx port (using "bit banging").

Additionally via a GPIO PWM Output the output power of the GTIL2 can be controlled. The setting is 
provided as parameter of the data request coming from the Control-PC.
