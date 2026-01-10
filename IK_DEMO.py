import math

def plot_robotic_arm(L1, L2, L3, x, y, z):
    # To find base angle you take atan2(y,x) which will give the proper angle while taking quadrants into account.
    #theta_base = math.atan2(y, x)  # in radians
    print(f"Base angle (radians): {theta_base}")
    print(f"Base angle (degrees): {math.degrees(theta_base)}") # in degrees
    
    # Convert the 3d coordinates (x,y,z) into 2d coordinates for the planar arm (x,y)
    planar_y = z # Need z to become y because z was the height of the arm's end effector
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

    # Compute shoulder for each candidate
    beta_1 = math.atan2(L2 * math.sin(theta_elbow_1), L1 + L2 * math.cos(theta_elbow_1))
    theta_shoulder_1 = alpha - beta_1

    beta_2 = math.atan2(L2 * math.sin(theta_elbow_2), L1 + L2 * math.cos(theta_elbow_2))
    theta_shoulder_2 = alpha - beta_2
    
    if theta_shoulder_1 >= 0:
        theta_shoulder,
    if theta_shoulder_2 >= 0:
        candidates.append((theta_shoulder_2, theta_elbow_2))

    if not candidates:
        print("No IK solution satisfies the shoulder constraint (theta_shoulder >= 0).")
        return

    # Calculating angle from the shoulder to the wrist point
    alpha = math.atan2(wrist_y, wrist_x)
    # Angle inside the triangle to the shoulder
    beta = math.atan2(L2*math.sin(theta_elbow), L1 + L2*math.cos(theta_elbow))

    theta_shoulder = alpha - beta # Shoulder angle
    
    # In order to get the probe pointing down always we need all the angles added together to be -pi/2 radians
    # This makes the probe always pointing straight down
    # Shoulder angle + Elbow angle + Wrist angle = -pi/2
    theta_wrist = -math.pi / 2  - theta_shoulder - theta_elbow
    
    print(f"Base angle (radians): {theta_base}")
    print(f"Base angle (degrees): {math.degrees(theta_base)}")
    print(f"Shoulder angle (radians): {theta_shoulder}")
    print(f"Shoulder angle (degrees): {math.degrees(theta_shoulder)}")
    print(f"Elbow angle (radians): {theta_elbow}")
    print(f"Elbow angle (degrees): {math.degrees(theta_elbow)}")
    print(f"Wrist angle (radians): {theta_wrist}")
    print(f"Wrist angle (degrees): {math.degrees(theta_wrist)}")

print(math.atan2(5, 5) * 180 / math.pi) #45 degrees
print(math.atan2(5, -5) * 180 / math.pi) #135 degrees
print(360 + math.atan2(-5, -5) * 180 / math.pi) #225 degrees
print(360 + math.atan2(-5, 5) * 180 / math.pi) #315 degrees

