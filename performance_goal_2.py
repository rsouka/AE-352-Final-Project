from final_project import *

t2, s2 = run_circle_test()

plt.figure(figsize=(15, 5))

# Top-down view
plt.subplot(1, 3, 1)
plt.plot(s2[:, 0], s2[:, 1], color='red', label='Actual')
ref_circle_x = 2.0 * np.cos(np.linspace(0, 2*np.pi, 100))
ref_circle_y = 2.0 * np.sin(np.linspace(0, 2*np.pi, 100))
plt.plot(ref_circle_x, ref_circle_y, 'k--', label='Desired')
plt.title('x-y trajectory')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.grid()
plt.legend()

# Altitude view
plt.subplot(1, 3, 2)
plt.plot(t2, s2[:, 2], label='Actual', color='red')
plt.axhline(1.0, color='k', linestyle='--', label='Desired (z = 1m)')
plt.ylim(0, 2)
plt.title('Altitude')
plt.xlabel('Time (s)'); 
plt.ylabel('z (m)')
plt.grid()
plt.legend()

# Euler angles
plt.subplot(1, 3, 3)
plt.title('Euler Angles')
plt.plot(t2, s2[:,3], label='Roll', color='blue')
plt.plot(t2, s2[:,4], label='Pitch', color='green')
plt.plot(t2, s2[:,5], label='Yaw', color='mediumvioletred')
plt.legend()
plt.ylim(-0.1, 0.1)
plt.grid(alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')

plt.suptitle('Performance Goal 2 (Circular Flight)', fontsize=16)
plt.tight_layout()


plt.savefig('PG2-1.png', dpi=400)


# Speed
plt.figure(figsize=(5,4))
speed = np.linalg.norm(s2[:, 6:9], axis=1)
plt.plot(t2, speed, label='Actual', color='red')
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.axhline(y=0.5, label='Target (0.5 m/s)', linestyle='--', color='black')
plt.legend()
plt.title('Drone speed during circular flight')

plt.savefig('PG2-2.png', dpi=400)