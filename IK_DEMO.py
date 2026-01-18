import math

def wrap_rad(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def plot_robotic_arm(L1, L2, L3, x, y, z, z_offset=0):
    # To find base angle you take atan2(y,x) which will give the proper angle while taking quadrants into account.
    theta_base = math.atan2(y, x)  # in radians
    print(f"Base angle (radians): {theta_base}")
    print(f"Base angle (degrees): {math.degrees(theta_base)}") # in degrees
    
    # Convert the 3d coordinates (x,y,z) into 2d coordinates for the planar arm (x,y)
    planar_y = z - z_offset # Need z to become y because z was the height of the arm's end effector
    planar_x = math.hypot(x, y) # The distance from origin to the projection of (x,y) on the ground plane
    print(f"Planar x: {planar_x}")
    print(f"Planar y: {planar_y}")
    
    # To find where the wrist joint needs to be placed we need to subtract the length of the last link (L3) from the planar coordinates
    wrist_x = planar_x # With the wrist pointing straight down the x coordinate does not change
    wrist_y = planar_y + L3  # With the wrist pointing straight down we do some calculations
    # wrist y = planar_y - L3 * sin(-pi/2) -> wrist_y = planar_y + L3
    
    # Checking to make sure that we can reach the point
    distance = math.hypot(wrist_x, wrist_y)
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        print("The point is out of reach for the robotic arm.")
        return
    
    c2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))  # clamp for safety
    s2 = math.sqrt(1 - c2**2)
    
    # Two possible elbow angles (elbow-up / elbow-down)
    theta_elbow_1 = math.atan2(+s2, c2)
    theta_elbow_2 = math.atan2(-s2, c2)
    
    # Calculating angle from the shoulder to the wrist point
    alpha = math.atan2(wrist_y, wrist_x)
    # Compute shoulder for each candidate
    beta_1 = math.atan2(L2 * math.sin(theta_elbow_1), L1 + L2 * math.cos(theta_elbow_1))
    theta_shoulder_1 = alpha - beta_1

    beta_2 = math.atan2(L2 * math.sin(theta_elbow_2), L1 + L2 * math.cos(theta_elbow_2))
    theta_shoulder_2 = alpha - beta_2
    
    if theta_shoulder_1 >= 0 and theta_shoulder_2 >= 0:
        y_elbow1 = L1 * math.sin(theta_shoulder_1)
        y_elbow2 = L1 * math.sin(theta_shoulder_2)
        if y_elbow1 > y_elbow2:
            theta_shoulder = theta_shoulder_1
            theta_elbow = theta_elbow_1
        else:
            theta_shoulder = theta_shoulder_2
            theta_elbow = theta_elbow_2
    elif theta_shoulder_1 >= 0:
        theta_shoulder = theta_shoulder_1
        theta_elbow = theta_elbow_1
    elif theta_shoulder_2 >= 0:
        theta_shoulder = theta_shoulder_2
        theta_elbow = theta_elbow_2
    else: 
        print("No valid shoulder angle found.")
        return
    
    # In order to get the probe pointing down always we need all the angles added together to be -pi/2 radians
    # This makes the probe always pointing straight down
    # Shoulder angle + Elbow angle + Wrist angle = -pi/2
    theta_wrist = -math.pi / 2  - theta_shoulder - theta_elbow

    map_base = wrap_rad(math.pi/2 - theta_base)
    map_shoulder = wrap_rad(math.pi/2 - theta_shoulder)
    map_elbow = wrap_rad(math.pi/2 - theta_elbow)
    map_wrist = wrap_rad(math.pi/2 - theta_wrist)

    
    print(f"Base angle (radians): {map_base}")
    print(f"Base angle (degrees): {math.degrees(map_base)}")
    print(f"Shoulder angle (radians): {map_shoulder}")
    print(f"Shoulder angle (degrees): {math.degrees(map_shoulder)}")
    print(f"Elbow angle (radians): {map_elbow}")
    print(f"Elbow angle (degrees): {math.degrees(map_elbow)}")
    print(f"Wrist angle (radians): {map_wrist}")
    print(f"Wrist angle (degrees): {math.degrees(map_wrist)}")

    return (theta_base, theta_shoulder, theta_elbow, theta_wrist), (map_base, map_shoulder, map_elbow, map_wrist)

def forward_kinematics(L1, L2, L3, theta_base, theta_shoulder, theta_elbow, theta_wrist, z_offset=0):
    # Planar wrist position
    xw = L1*math.cos(theta_shoulder) + L2*math.cos(theta_shoulder + theta_elbow)
    yw = L1*math.sin(theta_shoulder) + L2*math.sin(theta_shoulder + theta_elbow)

    # Tool pitch in the planar slice
    gamma = theta_shoulder + theta_elbow + theta_wrist  # should be -pi/2

    # Planar tip position
    xt = xw + L3*math.cos(gamma)
    yt = yw + L3*math.sin(gamma)

    # Back to 3D (undo planar_x = hypot(x,y))
    x = xt * math.cos(theta_base)
    y = xt * math.sin(theta_base)
    z = yt + z_offset

    return x, y, z, gamma


def move_relative(dx, dy, dz):
    global x_current, y_current, z_current, L1, L2, L3, z_offset
    x_current += dx
    y_current += dy
    z_current += dz
    return plot_robotic_arm(L1, L2, L3, x_current, y_current, z_current, z_offset=z_offset)













# L1, L2, L3 = 10, 10, 2

# tests = [
#     (10, 0, 0, 0),     # (x,y,z,z_offset)
#     (7, 7, 0, 0),
#     (10, 0, 5, 0),
#     (10, 0, 0, 5),     # shoulder 5 inches above table
# ]

# for x, y, z, z_off in tests:
#     print("\n--- TEST ---")
#     print("Target:", (x, y, z), "z_offset:", z_off)

#     sol = plot_robotic_arm(L1, L2, L3, x, y, z, z_offset=z_off)
#     if sol is None:
#         continue

#     raw, mapped = sol

#     tb, ts, te, tw = raw
#     x2, y2, z2, gamma = forward_kinematics(L1, L2, L3, tb, ts, te, tw, z_offset=z_off)

#     print("FK result:", (x2, y2, z2))
#     print("Position error:", (x2-x, y2-y, z2-z))
#     print("Gamma (deg):", math.degrees(gamma))  # should be ~ -90


# print(math.atan2(5, 5) * 180 / math.pi) #45 degrees
# print(math.atan2(5, -5) * 180 / math.pi) #135 degrees
# print(360 + math.atan2(-5, -5) * 180 / math.pi) #225 degrees
# print(360 + math.atan2(-5, 5) * 180 / math.pi) #315 degrees