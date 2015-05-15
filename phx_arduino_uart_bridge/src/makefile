test: test.cpp
	g++ -std=c++0x -o test test.cpp serial_com.cpp -I "../include/" -fno-stack-protector

prestart: phx_arduino_uart_bridge_prestart.cpp
	g++ -std=c++0x -o prestart phx_arduino_uart_bridge_prestart.cpp serial_com.cpp -I "../include/"
