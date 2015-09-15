test: test.cpp
	g++ -std=c++0x -o test test.cpp serial_com.cpp -I "../include/"

test_gps: test_gps.cpp
	g++ -std=c++0x -o test_gps test_gps.cpp serial_com.cpp -I "../include/"

prestart: phx_arduino_uart_bridge_prestart.cpp
	g++ -std=c++0x -o prestart phx_arduino_uart_bridge_prestart.cpp serial_com.cpp -I "../include/"
