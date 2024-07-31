WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82


def checkLimitLinearVelocity(vel: float) -> float:
    if vel < -WAFFLE_MAX_LIN_VEL:
        return -WAFFLE_MAX_LIN_VEL
    if vel > WAFFLE_MAX_LIN_VEL:
        return WAFFLE_MAX_LIN_VEL
    
    return vel

def checkLimitAngularVelocity(vel: float) -> float:
    if vel < -WAFFLE_MAX_ANG_VEL:
        return -WAFFLE_MAX_ANG_VEL
    if vel > WAFFLE_MAX_ANG_VEL:
        return WAFFLE_MAX_ANG_VEL
    
    return vel
