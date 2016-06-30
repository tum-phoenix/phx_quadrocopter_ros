# the message the Autopilot sends to the Controllers
# currently used so the Autopilot can disable Controllers/Nodes
Header header
# the node the message is directed at; other nodes should ignore this message
# see autopilot.py file for number
int16 node_identifier
# used to disable the specified node
bool[] enabled
