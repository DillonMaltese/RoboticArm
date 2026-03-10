import math

def wrap_rad(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def wrap_deg(a):
    return (a + 180) % 360 - 180

def find_angles(L1, L2, L3, X, Y, Z, z_offset=0):
    # Base angle to rotate towards the target in the XY plane
    theta_base = math.atan2(Y, X)
    
    x_plane = math.sqrt(X**2 + Y**2)
    y_plane = Z - z_offset
    
    x_wrist = x_plane
    y_wrist = y_plane + L3  # account for wrist length to keep probe pointing down
    
    d = math.sqrt(x_wrist**2 + y_wrist**2)
    
    if abs(L1 - L2) <= d <= (L1 + L2):
        # Law of cosines for elbow angle
        c2 = max(-1.0, min(1.0, (d**2 - L1**2 - L2**2) / (2 * L1 * L2)))
        theta_2 = math.atan2(math.sqrt(1 - c2**2), c2)  # elbow-down solution
        alpha = math.atan2(y_wrist, x_wrist)
        beta = math.atan2(L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2))
        theta_1 = alpha - beta
        theta_3 = -math.pi/2 - theta_1 - theta_2  # wrist angle to keep probe pointing down
        return wrap_rad(theta_base), wrap_rad(theta_1), wrap_rad(theta_2), wrap_rad(theta_3)
    else:
        return None
    
    
def travel_distance(current_deg, target_deg):
    return wrap_deg(target_deg - current_deg)


def map_angles(base_rad, shoulder_rad, elbow_rad, wrist_rad):
    base_deg     = math.degrees(base_rad)
    shoulder_deg = -(math.degrees(shoulder_rad) - 90)
    elbow_deg    = -(math.degrees(elbow_rad) - 90)
    wrist_deg    = -(math.degrees(wrist_rad) - 90)
    return base_deg, shoulder_deg, elbow_deg, wrist_deg