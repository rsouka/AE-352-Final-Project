from final_project import *

t3, s3 = run_mission_test()


# 3D view
fig = plt.figure(figsize=(12, 8), tight_layout=True)
ax = fig.add_subplot(111, projection='3d')
ax.plot(s3[:, 0], s3[:, 1], s3[:, 2], linewidth=2, label='Drone Path', color='red')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

ax.scatter(0, 0, 0, color='green', label='Start')
ax.scatter(5, 5, 0, color='blue', label='End')

ax.set_box_aspect((1,1,0.2), zoom=0.8)

ax.legend(bbox_to_anchor=(0.9, 0.8))
ax.set_title('Mission Trajectory',y=0.85)

plt.savefig('PG3-1.png', dpi=400)



# Landing velocity
plt.figure(figsize=(6,3), tight_layout=True)

landing_mask = t3 > 19.0
if np.any(landing_mask):
    # State index 8 is vel_z
    plt.plot(t3[landing_mask], s3[landing_mask, 8], label='Vertical Velocity', color='red')
    plt.axhline(-0.01, color='black', linestyle='--', label='1 cm/s maximum')
    plt.title('Landing Velocity')
    plt.ylabel('Velocity (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.legend()

plt.savefig('PG3-2.png', dpi=400)





# Euler angles
plt.figure(figsize=(6,4), tight_layout=True)
plt.plot(t3, s3[:,3], label='Roll', color='blue')
plt.plot(t3, s3[:,4], label='Pitch', color='green')
plt.plot(t3, s3[:,5], label='Yaw', color='mediumvioletred')
plt.grid()
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Euler angles')
plt.savefig('PG3-3.png', dpi=400)



# Straight section velocity

plt.figure(figsize=(6,4), tight_layout=True)

speed = np.linalg.norm(s3[:, 6:9], axis=1)
plt.plot(t3, speed, label='Actual velocity', color='red')
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.title('Drone speed during first straight section')

plt.axhline(y=1, linestyle='--', color='black', label='Target: 1 m/s average')

avg = np.trapezoid(speed[np.where((t3 >= 2) & (t3 <= 7))]) / len(np.where((t3 >= 2) & (t3 <= 7))[0])
plt.axhline(y=avg, color='blue', label=f'Actual: {avg:.2f} m/s average')

plt.xlim(2,7)
plt.legend()

plt.savefig('PG3-4.png', dpi=400)