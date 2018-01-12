@echo off
batchisp -device AT32UC3A0512 -hardware usb -operation onfail abort memory FLASH erase F loadbuffer "analog_signal_slave\Debug\analog_signal_slave.hex" program verify start reset 0
pause