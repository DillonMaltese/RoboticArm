# from IK import *
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# L1, L2, L3 = 28.5, 16, 11
# Z_OFFSET = 0

# fig = plt.figure(figsize=(8, 8))
# ax = fig.add_subplot(111, projection='3d')

# base_rad = math.radians(0)
# shoulder_rad = math.radians(90)
# elbow_rad = math.radians(-90)
# wrist_rad = math.radians(-90)

# def get_positions(base_rad, shoulder_rad, elbow_rad, wrist_rad):
#     base = (0, 0, 0)
#     shoulder = (
#         L1 * math.cos(shoulder_rad) * math.cos(base_rad),
#         L1 * math.cos(shoulder_rad) * math.sin(base_rad),
#         L1 * math.sin(shoulder_rad)
#     )
#     elbow_angle = shoulder_rad + elbow_rad
#     elbow = (
#         shoulder[0] + L2 * math.cos(elbow_angle) * math.cos(base_rad),
#         shoulder[1] + L2 * math.cos(elbow_angle) * math.sin(base_rad),
#         shoulder[2] + L2 * math.sin(elbow_angle)
#     )
#     wrist_angle = elbow_angle + wrist_rad
#     probe = (
#         elbow[0] + L3 * math.cos(wrist_angle) * math.cos(base_rad),
#         elbow[1] + L3 * math.cos(wrist_angle) * math.sin(base_rad),
#         elbow[2] + L3 * math.sin(wrist_angle)
#     )
#     xs = [base[0], shoulder[0], elbow[0], probe[0]]
#     ys = [base[1], shoulder[1], elbow[1], probe[1]]
#     zs = [base[2], shoulder[2], elbow[2], probe[2]]
#     return xs, ys, zs

# def plot_arm(ax, angles_rad, color, label):
#     xs, ys, zs = get_positions(*angles_rad)
#     ax.plot(xs, ys, zs, color=color, marker='o', linewidth=3, markersize=8, label=label)

# fig = plt.figure(figsize=(8, 8))
# ax = fig.add_subplot(111, projection='3d')

# # home position
# home_x, home_y, home_z = 0, 0.0, L1
# home_angles = find_angles(L1, L2, L3, home_x, home_y, home_z)
# plot_arm(ax, home_angles, 'blue', 'home')

# # 3 inches forward
# fwd_angles = find_angles(L1, L2, L3, home_x + 3, home_y, home_z)
# plot_arm(ax, fwd_angles, 'red', '3 inches forward')

# ax.set_xlabel('X (inches)')
# ax.set_ylabel('Y (inches)')
# ax.set_zlabel('Z (inches)')
# ax.set_title('Robotic Arm - Home vs 3 inches forward')
# ax.set_xlim(-5, 50)
# ax.set_ylim(-30, 30)
# ax.set_zlim(0, 40)
# ax.legend()

# plt.tight_layout()
# plt.show()

# print("Home angles (degrees):", [math.degrees(a) for a in home_angles])
# print("Fwd angles (degrees):", [math.degrees(a) for a in fwd_angles])

# # Test: what angles does the IK give for a simple known position?
# # Arm straight out horizontally at x=20, y=0, z=17.5
# # angles = find_angles(28.5, 16, 11, 20, 0, 17.5)
# # print("Raw radians:", angles)
# # print("Degrees:", map_angles(*angles))

# # # What does moving forward 3 inches give?
# # angles2 = find_angles(28.5, 16, 11, 23, 0, 17.5)
# # print("\nAfter 3 inches forward:")
# # print("Raw radians:", angles2)
# # print("Degrees:", map_angles(*angles2))

# # # What are the deltas?
# # d0 = travel_distance(map_angles(*angles)[0], map_angles(*angles2)[0])
# # d1 = travel_distance(map_angles(*angles)[1], map_angles(*angles2)[1])
# # d2 = travel_distance(map_angles(*angles)[2], map_angles(*angles2)[2])
# # d3 = travel_distance(map_angles(*angles)[3], map_angles(*angles2)[3])
# # print(f"\nDeltas: base={d0:.2f}, shoulder={d1:.2f}, elbow={d2:.2f}, wrist={d3:.2f}")


from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from old.IK import find_angles
import matplotlib.pyplot as plt

my_chain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(name="base",     origin_translation=[0,0,0],    origin_orientation=[0,0,0], rotation=[0,0,1]),
    URDFLink(name="shoulder", origin_translation=[0,0,28.5], origin_orientation=[0,0,0], rotation=[0,1,0]),
    URDFLink(name="elbow",    origin_translation=[16,0,0],   origin_orientation=[0,0,0], rotation=[0,1,0]),
    URDFLink(name="wrist",    origin_translation=[11,0,0],   origin_orientation=[0,0,0], rotation=[0,1,0]),
])

home_angles = find_angles(28.5, 16, 11, 16, 0, 17.5)
fwd_angles  = find_angles(28.5, 16, 11, 26, 0, 17.5)

base_h, shoulder_h, elbow_h, wrist_h = home_angles
base_f, shoulder_f, elbow_f, wrist_f = fwd_angles

ikpy_home = [0, base_h, shoulder_h, elbow_h, wrist_h]
ikpy_fwd  = [0, base_f, shoulder_f, elbow_f, wrist_f]

fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
my_chain.plot(ikpy_home, ax)
my_chain.plot(ikpy_fwd,  ax)

ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_zlim(0, 40)
plt.legend(['home', '3 inches forward'])
plt.show()