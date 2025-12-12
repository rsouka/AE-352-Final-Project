from final_project import *

t1, s1 = run_hover_test()


# Altitude
plt.figure(figsize=(10, 3))
plt.plot(t1, s1[:, 2], label='Actual', zorder=10, color='red') # Plot Z altitude
plt.axhline(1.0, color='k', linestyle='--', linewidth=2, label='Target')
plt.title('Goal 1: Hover Altitude (z position)')
plt.grid(); plt.xlabel('Time (s)'); plt.ylabel('Altitude (m)')
plt.legend()
plt.tight_layout()
plt.savefig('PG1-1.png', dpi=400)




# Euler angles and x-y trajectory
plt.figure(figsize=(12, 3))
plt.subplot(1,2,1)
plt.plot(t1, s1[:,3], label='Roll', color='blue')
plt.plot(t1, s1[:,4], label='Pitch', color='green')
plt.plot(t1, s1[:,5], label='Yaw', color='mediumvioletred')
plt.legend()
plt.ylim(-0.1, 0.1)
plt.grid(alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Euler angles')

plt.subplot(1,2,2)
plt.plot(t1, s1[:, 0], label='x', color='blue')
plt.plot(t1, s1[:,1], label='y', color='green')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.grid()
plt.title('x-y trajectory')

plt.tight_layout()
plt.savefig('PG1-2.png', dpi=400)