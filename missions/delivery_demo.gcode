; Project 006-D - 0.5 lb delivery demo
; Units: metres. X = North, Y = East, Z = height above the takeoff pad.
; Validate first:  python -m project006.grid_commander missions/delivery_demo.gcode --dry-run

M17 Z5              ; arm + take off to 5 m (hook is locked automatically)
M114                ; print position
G0 X15 F3           ; fly 15 m north at 3 m/s
G0 Y10 A90          ; 10 m east, face east
G0 Z3               ; descend to 3 m drop height (vertical speed = WP_SPD_DN)
G4 S2               ; let the payload stop swinging
M100                ; RELEASE the 0.5 lb payload
G4 S1
G0 Z6               ; climb away from the drop
G28                 ; return to launch and land
