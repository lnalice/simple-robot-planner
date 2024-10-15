"""
@param linear_velocity.x , angular_velocity.z
@return FRONT | RIGHT | BACK | LEFT
"""
def vel2statusByte (linX: float, angZ: float) -> int:
    led_pins = {'F': 0b0001, 'R': 0b0010, 'B': 0b0100, 'L': 0b1000, 'A': 0b1111} # front right back left (시계방향)

    if angZ > 0.04: # LEFT
        return led_pins['L']
    if angZ < -0.04: # RIGHT
        return led_pins['R']
    if linX > 0: # FRONT
        return led_pins['F']
    if linX < 0: # BACK
        return led_pins['B']
    
    return led_pins['A']  # spin