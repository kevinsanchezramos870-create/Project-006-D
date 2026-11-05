; 20 m square at 10 m, relative moves (G91), with an operator pause.
M17 Z10
G91                 ; relative: each move is added to the last position
G1 X20 F4
G1 Y20
G1 X-20
G1 Y-20
G90                 ; back to absolute
M0 Check the video feed, then press Enter
G0 X0 Y0 Z10 A0
M18                 ; land right here
