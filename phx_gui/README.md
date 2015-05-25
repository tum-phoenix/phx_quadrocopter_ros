Die Serielle Kommunikation mit den beiden MicroControllern läuft derzeit in einem Python-Skript ab welches die Objekte der pyCopter Klasse nutzt.

pyCopter
	.serial_com
		.multiwii_portocoll
			Baut eine serielle Verbindung zu Multiwii oder onboard Arduino auf
			und kümmert sich um die Serielle Kommunikation und den Austausch von Daten
		.serial_rc
			gegenwärtig nicht in Verwendung und wird wohl rausfliegen da die betreffenden
			methoden schon in multiwii_protocoll eingebaut sind.
	.network_com
		.osc_receiver
			Horcht nach an die eigene IP gerichtete Nachrichten und leitet sie an
			das richtige Modul weiter, zum Beispiel eine OSC RC.
		.osc_transmitter
			Sendet die Flugdaten ins Netzwerk und hofft darauf dass sie vom
			MonitoringPC empfangen werden.
		.osc_rc
			Geht mit eingehenden OSC RC Daten um und wandelt sie in ein für die 
			Serielle Kommunikation brauchbares Format um.
	.copter_status
		Dies ist das Objekt welches serielle und Netzwerkverbindungen sowie sämtliche 
		Flugdaten zusammenfasst. Die Methoden der copter_status Klasse sollten im Normalfall
		ausreichen die Gewünschten Befehle aus dem Netzwerk zu erhalten bzw. an das MultiWii
		zu senden.
	.virtual_remote_control
		Dieses Objekt ist noch in der Entwicklung und soll helfen die Steuerung per PC zu
		erleichtern indem verschiedenste Kommandos interpretiert und richtig fusioniert
		werden. Außerdem finden hier die Rechnungen statt die die Steuerungsdaten für den
		seriellen Ausgang brauchbar macht.
