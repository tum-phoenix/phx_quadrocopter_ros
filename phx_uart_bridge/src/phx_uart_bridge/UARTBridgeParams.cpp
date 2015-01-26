/*
 * UARTInterfaceParams.cpp
 *
 *  Created on: 13.07.2014
 *      Author: ic3
 */

#include <phx_uart_bridge/UARTBridgeParams.h>

const std::string UARTBridgeParams::PATH = "phx_uart_bridge/path";
const std::string UARTBridgeParams::PATH_DEFAULT = "/dev/ttyUSB0";
const std::string UARTBridgeParams::BAUDRATE = "phx_uart_bridge/baudrate";

const std::string UARTBridgeParams::TOPIC_MOTOR = "/tum_phoenix/qc/motor_data";
const std::string UARTBridgeParams::TOPIC_MOTOR_RAW = "/tum_phoenix/qc/motor_data/raw";

const std::string UARTBridgeParams::TOPIC_SENSOR = "/tum_phoenix/qc/sensor_data";
const std::string UARTBridgeParams::TOPIC_SENSOR_RAW = "/tum_phoenix/qc/sensor_data/raw";
