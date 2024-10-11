"""
@param linear_velocity.x , angular_velocity.z
@return FRONT | RIGHT | BACK | LEFT
"""
def vel2statusByte (linX: float, angZ: float) -> bytes:
    led_pins = ['F', 'R', 'B', 'L'] # front right back left (시계방향)

    if angZ > 0.2: # LEFT
        return bin(1 << led_pins.index('L'))
    if angZ < -0.2: # RIGHT
        return bin(1 << led_pins.index('R'))
    if linX > 0: # FRONT
        return bin(1 << led_pins.index('F'))
    if linX < 0: # BACK
        return bin(1 << led_pins.index('B'))
    
    return '0b1111'  # spin