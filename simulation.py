import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# 1. MATH HELPERS (Same as before)
# ================================
def get_cylinder_mesh(radius, width, n_steps=20):
    theta = np.linspace(0, 2*np.pi, n_steps)
    z = np.linspace(-width/2, width/2, 2)
    Theta, Z = np.meshgrid(theta, z)
    X = radius * np.cos(Theta)
    Y = radius * np.sin(Theta)
    return X, Y, Z

def apply_transform(X, Y, Z, R, pos):
    points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()])
    rotated = R @ points
    X_new = rotated[0, :].reshape(X.shape) + pos[0]
    Y_new = rotated[1, :].reshape(Y.shape) + pos[1]
    Z_new = rotated[2, :].reshape(Z.shape) + pos[2]
    return X_new, Y_new, Z_new

def rotation_matrix(yaw, lean, spin):
    # Spin (Y axis local)
    c, s = np.cos(spin), np.sin(spin)
    R_spin = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    
    # Lean (X axis local - Rolling)
    c, s = np.cos(lean), np.sin(lean)
    R_lean = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    
    # Yaw (Z axis global)
    c, s = np.cos(yaw), np.sin(yaw)
    R_yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    
    return R_yaw @ R_lean @ R_spin

# 2. SCENE SETUP (Fixed Camera)
# =============================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# FIX: Set static bounds large enough for the whole circle
ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_zlim(0, 8)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_aspect('equal')

# Add a marker for the center of the world
ax.scatter([0], [0], [0], color='black', s=100, marker='x', label='Center')

# Geometry Init
wheel_radius = 1.0
raw_X, raw_Z, raw_Y = get_cylinder_mesh(wheel_radius, 0.2) 

wheel_surf = ax.plot_surface(raw_X, raw_Y, raw_Z, color='gray', alpha=0.9)
frame_line, = ax.plot([], [], [], color='red', linewidth=4)
spoke_line, = ax.plot([], [], [], color='white', linewidth=2)

# 3. ANIMATION UPDATE (Circular Path)
# ===================================
def update(frame_idx):
    # Controls the speed of the simulation
    t = frame_idx * 0.05
    
    # --- PATH LOGIC: CIRCLE ---
    path_radius = 6.0
    
    # 1. Position (Polar coordinates)
    pos_x = path_radius * np.cos(t)
    pos_y = path_radius * np.sin(t)
    
    # 2. Yaw (Steering)
    # The unicycle faces the tangent of the circle.
    # Tangent of a circle is (theta + 90 degrees)
    yaw_angle = t + (np.pi / 2)
    
    # 3. Lean (Physics)
    # To balance in a circle, we must lean inward (towards the center).
    # Since we are turning Left, we lean Left.
    lean_angle = 0.3 # Constant lean into the turn
    
    # 4. Wheel Spin
    # Arc length s = r_path * theta
    distance_traveled = path_radius * t
    # Wheel rotation = -distance / r_wheel
    wheel_angle = -distance_traveled / wheel_radius

    # --- RENDER UPDATES ---
    pos = np.array([pos_x, pos_y, wheel_radius])
    R = rotation_matrix(yaw_angle, lean_angle, wheel_angle)
    
    # Update Wheel
    global wheel_surf
    wheel_surf.remove()
    X_world, Y_world, Z_world = apply_transform(raw_X, raw_Y, raw_Z, R, pos)
    wheel_surf = ax.plot_surface(X_world, Y_world, Z_world, color='#333333', shade=True)
    
    # Update Frame (No spin)
    R_frame = rotation_matrix(yaw_angle, lean_angle, 0)
    frame_local = np.array([[0, 0, 0], [0, 0, 2.5], [0, 0, 0]]) 
    frame_world = R_frame @ frame_local
    frame_line.set_data(frame_world[0,:] + pos[0], frame_world[1,:] + pos[1])
    frame_line.set_3d_properties(frame_world[2,:] + pos[2])
    
    # Update Spoke (With spin)
    spoke_local = np.array([[0, 0, 0], [wheel_radius, 0, 0], [0, 0, 0]])
    spoke_world = R @ spoke_local
    spoke_line.set_data(spoke_world[0,:] + pos[0], spoke_world[1,:] + pos[1])
    spoke_line.set_3d_properties(spoke_world[2,:] + pos[2])

# 4. RUN
ani = FuncAnimation(fig, update, frames=200, interval=30, blit=False)
plt.show()