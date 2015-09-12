Um mit MicroControllern über eine serielle UART Verbindung zu kommunizieren wird das dem Mavlink Protokoll angelehnte Protokoll Multiwii Serial Protocoll (msp) verwendet.

Nachrichten werden hier durch einen Request des Hosts (Computer) an den Client (micro controller) angefragt und eine Antwort wird vom Client geantwortet. Die Nachrichten bauen sich wie folgt auf:

```
> START_BYTE  PROTOCOL_BYTE  DIRECTION_BYTE  MSG_TYPE  MSG_LENGTH  Payload  CHECK_BYTE <

START_BYTE = '$'
PROTOCOL_BYTE = 'M' for msp or 'P' for phoenix
DIRECTION_BYTE = '>' or '<' or in case of an error '!'
MSG_TYPE = (uint8_t) well defined message code which is globaly fixed
MSG_LENGTH = (uint8_t) length of the payload in number of bytes
Payload = row of MSG_LENGTH bytes
CHECK_BYTE = (uint8_t) MSG_TYPE ^ MSG_LENGTH ^ Payload[i in range 0 to MSG_LENGTH]
```

Ein request einer Nachricht mit einem speziellen MSG_TYPE ist eine Nachricht dieses MSG_TYPE jedoch mit MSG_LENGTH = 0 und ohne jegliche Payload.

Die Serielle Kommunikation kann via Python-Skript mit der pyCopter Klasse genutzt werden. Dies ist jedoch nicht sonderlich effizient.

Daher wurde eine cpp Version zur Seriellen Kommunikation entwickelt um effizient und beliebig skalierbar Daten über die Serielle Schnittstelle auszutauschen. Das cpp Skript published die Daten dabei direkt in ROS, bzw. kann Nachrichten auch direkt an den MicroController weiterleiten.
