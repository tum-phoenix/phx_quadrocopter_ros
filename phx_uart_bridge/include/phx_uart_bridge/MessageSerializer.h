/*
 * MessageSerializer.h
 *
 *  Created on: 15.07.2014
 *      Author: ic3
 */

#ifndef MESSAGESERIALIZER_H_
#define MESSAGESERIALIZER_H_

#include <phx_uart_bridge/MotorMessage.h>
#include <phx_uart_bridge/SensorMessage.h>

class MessageSerializer {

public:
	static int serialize(phx_uart_bridge::MotorMessage & msg, char buffer[]);

	/**
	 * UART message sequence (receive)
	 *
	 *  1. 1 Byte: Start Flag (U)
	 *  2. 1 Byte: Radio Kanal 0: Remote indicator: 0 (OFF), 1 (ON)
	 *  3. 1 Byte: Radio Kanal 1
	 *  4. 1 Byte: Radio Kanal 2
	 *  5. 1 Byte: Radio Kanal 3
	 *  6. 2 Byte: Lineare Beschleunigung X
	 *  7. 2 Byte: Lineare Beschleunigung Y
	 *  8. 2 Byte: Lineare Beschleunigung Z
	 *  9. 2 Byte: Winkelgeschwindigkeit X
	 * 10. 2 Byte: Winkelgeschwindigkeit Y
	 * 11. 2 Byte: Winkelgeschwindigkeit Z
	 * 12. 1 Byte: Spannung Akku
	 *
	 */
	static void deserialize(phx_uart_bridge::SensorMessagePtr & msg, const char buffer[]);
};



#endif /* MESSAGESERIALIZER_H_ */
