# the message sent from the Controllers to the Autopilot
Header header
int16 node_identifier
int16 priority
geometry_msgs/Twist cmd
# rc: contains the values the sending controller wants to set for pitch/throttle/etc.
phx_uart_msp_bridge/RemoteControl rc
