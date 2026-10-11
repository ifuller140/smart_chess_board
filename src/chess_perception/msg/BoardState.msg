std_msgs/Header header
int8[64] pieces          # 8x8 flattened array (0=empty, 1=white pawn, etc.)
string fen               # FEN string representation
geometry_msgs/Point[4] corners # Board corners in image coordinates
