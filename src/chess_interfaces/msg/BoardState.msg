# Detected board geometry. header.stamp is the time of the last genuinely
# fresh detection (see board_detector_node.py) — compare against now() to
# tell a fresh detection from a stale cached republish.
std_msgs/Header header
geometry_msgs/Point[4] corners # Board corners in image coordinates, order TL/TR/BR/BL
