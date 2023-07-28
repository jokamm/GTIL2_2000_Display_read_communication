# GTIL2_2000_Display_read_communication

This SW is for Raspberry Pi Pico. It will be reading data from the LCD display communication of the
GTIL2 2000 (or 1000) and the RS232 communication. The data is buffered in the Pico, so it can be 
send on request. The request comes from a control-PC, using a ModBus protocol that is compatible with 
the ModBus protocol of the DALY BMS.
