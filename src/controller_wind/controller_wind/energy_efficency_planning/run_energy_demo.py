import numpy as np
import networkx as nx

# Fix: system python3-matplotlib (3.5) installs a stale mpl_toolkits that
# conflicts with the pip-installed matplotlib (3.8). Force the pip version.
import sys, types
_pip_mpl = '/home/mitchell/.local/lib/python3.10/site-packages/mpl_toolkits'
_mod = types.ModuleType('mpl_toolkits')
_mod.__path__ = [_pip_mpl]
_mod.__package__ = 'mpl_toolkits'
sys.modules['mpl_toolkits'] = _mod

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# =========================================================
# WIND FIELD - Separate from planner, can be swapped/updated
# =========================================================
class WindField:
    """Callable wind vector field. Query wind at any 3D position.
    
    Can be:
      - Built from analytic models (tunnels, vortices, etc.)
      - Populated from sensor measurements
      - Updated online as new data arrives
      - Replaced entirely between planning calls
    """
    def __init__(self):
        self._sources = []  # List of wind source functions
    
    def add_source(self, source_func):
        """Add a wind source: callable(pos) -> np.array([wx, wy, wz])"""
        self._sources.append(source_func)
        return self  # Allow chaining
    
    def clear(self):
        """Remove all wind sources (e.g. before rebuilding from new data)."""
        self._sources = []
    
    def query(self, pos):
        """Query total wind vector at position. Sums all sources."""
        wind = np.array([0.0, 0.0, 0.0])
        for src in self._sources:
            wind = wind + src(pos)
        return wind
    
    def __call__(self, pos):
        """Make the object directly callable for convenience."""
        return self.query(pos)


def make_cylinder_source(center, axis, radius, speed):
    """Factory: creates a cylindrical wind source function.
    
    Args:
        center: 3D point on the cylinder axis
        axis:   direction vector (will be normalized) — also the wind direction
        radius: cylinder radius in meters
        speed:  wind speed in m/s
    """
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    center = np.array(center, dtype=float)
    
    def source(pos):
        p = np.array(pos) - center
        proj = np.dot(p, axis) * axis
        perp_dist = np.linalg.norm(p - proj)
        if perp_dist <= radius:
            return speed * axis
        return np.array([0.0, 0.0, 0.0])
    
    return source

class EnergyPlanner3D:
    def __init__(self, start, goal, v_cruise=20.0, wind_field=None, safety_margin=0.75):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.v_cruise = v_cruise
        self.wind_field = wind_field  # WindField object (or any callable(pos) -> [wx,wy,wz])
        self.safety_margin = safety_margin  # meters — clearance buffer around headwind zones
        
        # Grid settings (Coarse grid for speed in this demo)
        self.resolution = 0.25
        self.bounds_x = (-3, 3)
        self.bounds_y = (-3, 3)
        self.bounds_z = (0, 4)

    def set_wind_field(self, wind_field):
        """Update the wind field (e.g. after new sensor data). Allows replanning."""
        self.wind_field = wind_field

    def get_wind(self, pos):
        """Query wind at position. Returns zero if no wind field is set."""
        if self.wind_field is not None:
            return self.wind_field(pos)
        return np.array([0.0, 0.0, 0.0])

    def get_edge_cost(self, p1, p2):
        dist = np.linalg.norm(p2 - p1)
        direction = (p2 - p1) / dist
        
        # Sample wind at midpoint
        midpoint = (p1 + p2) / 2
        wind = self.get_wind(midpoint)
        
        # Project wind onto flight direction
        wind_help = np.dot(wind, direction)
        
        # Effective Ground Speed
        # If headwind exceeds cruise speed, cost is infinite (can't make progress)
        if wind_help < -self.v_cruise + 1.0: 
            return float('inf')
        
        # Ground speed = airspeed + wind component along travel
        ground_speed = self.v_cruise + wind_help
        
        # Energy cost ~ time = distance / ground_speed
        # Tailwind increases ground_speed → lower cost (naturally exploits)
        # Headwind decreases ground_speed → higher cost (naturally avoids)
        time_cost = dist / ground_speed
        
        # SAFETY MARGIN — probe nearby points to penalize proximity to headwind
        # This keeps the drone away from wind zone boundaries even if the
        # wind field has a hard edge. The wind field stays accurate; the
        # planner adds its own caution.
        if self.safety_margin > 0:
            probe_dirs = [
                np.array([1,0,0]), np.array([-1,0,0]),
                np.array([0,1,0]), np.array([0,-1,0]),
                np.array([0,0,1]), np.array([0,0,-1]),
            ]
            worst_headwind = 0.0
            for pd in probe_dirs:
                probe_pos = p2 + self.safety_margin * pd
                probe_wind = self.get_wind(probe_pos)
                probe_headwind = -np.dot(probe_wind, direction)  # positive = headwind
                worst_headwind = max(worst_headwind, probe_headwind)
            
            if worst_headwind > 0.1:
                # Penalize: nearby headwind detected within safety margin
                # Scale: penalty proportional to how strong that headwind is
                # relative to cruise speed
                penalty = 1.0 + 2.0 * (worst_headwind / self.v_cruise)
                time_cost *= penalty
        
        # WIND CONSISTENCY BONUS
        # Prefer nodes surrounded by consistent wind (center of flow)
        # over nodes at the edge (turbulent boundary).
        # Sample wind at p2 and its immediate neighborhood, compare magnitudes.
        wind_at_p2 = self.get_wind(p2)
        wind_mag = np.linalg.norm(wind_at_p2)
        
        if wind_mag > 0.1:  # Only apply if there's meaningful wind at this node
            # Sample 6 cardinal neighbors around p2
            probe_offsets = [
                np.array([1,0,0]), np.array([-1,0,0]),
                np.array([0,1,0]), np.array([0,-1,0]),
                np.array([0,0,1]), np.array([0,0,-1]),
            ]
            probe_dist = self.resolution * 0.5
            
            consistency = 0.0
            for off in probe_offsets:
                neighbor_wind = self.get_wind(p2 + probe_dist * off)
                neighbor_mag = np.linalg.norm(neighbor_wind)
                # How similar is the neighbor's wind to ours?
                consistency += min(neighbor_mag / wind_mag, 1.0)
            
            # consistency: 0 (edge, no neighbors have wind) to 6 (fully surrounded)
            # Normalize to [0, 1]
            consistency /= len(probe_offsets)
            
            # Reward being in the center: up to 30% cost reduction
            # Edge nodes (consistency ~0.5) get little/no benefit
            center_reward = 0.3 * max(0.0, consistency - 0.5) * 2.0  # 0 at edge, 0.3 at center
            
            # Check if wind is helpful (tailwind) or harmful (headwind)
            if wind_help > 0:
                # Tailwind: reward being deep inside the flow
                time_cost *= (1.0 - center_reward)
            else:
                # Headwind: penalize being deep inside (harder to escape)
                time_cost *= (1.0 + center_reward)
        
        return max(0.01, time_cost)

    def find_path(self):
        # 1. Generate Nodes
        xs = np.arange(self.bounds_x[0], self.bounds_x[1]+1, self.resolution)
        ys = np.arange(self.bounds_y[0], self.bounds_y[1]+1, self.resolution)
        zs = np.arange(self.bounds_z[0], self.bounds_z[1]+1, self.resolution)
        
        G = nx.DiGraph()
        
        # 2. Build Graph (Neighbor connections)
        # We allow 26-connectivity (neighbors in 3D cube)
        offsets = [-1, 0, 1]
        
        # Helper to get node ID
        def get_id(p): return tuple(np.round(p, 1))

        # Create nodes first to ensure existence
        all_points = []
        for x in xs:
            for y in ys:
                for z in zs:
                    all_points.append(np.array([x,y,z]))
                    
        print("Building Graph (this might take a moment)...")
        
        for x in xs:
            for y in ys:
                for z in zs:
                    u = (x,y,z)
                    # Check 6 primary neighbors + diagonals
                    for dx in offsets:
                        for dy in offsets:
                            for dz in offsets:
                                if dx==0 and dy==0 and dz==0: continue
                                
                                v = (x+dx, y+dy, z+dz)
                                # Bounds check
                                if (self.bounds_x[0] <= v[0] <= self.bounds_x[1] and
                                    self.bounds_y[0] <= v[1] <= self.bounds_y[1] and
                                    self.bounds_z[0] <= v[2] <= self.bounds_z[1]):
                                    
                                    cost = self.get_edge_cost(np.array(u), np.array(v))
                                    if cost != float('inf'):
                                        G.add_edge(u, v, weight=cost)

        # 3. Run A*
        print("Running A*...")
        # Find nearest grid nodes to start/goal
        start_node = min(G.nodes, key=lambda n: np.linalg.norm(np.array(n)-self.start))
        goal_node = min(G.nodes, key=lambda n: np.linalg.norm(np.array(n)-self.goal))
        
        try:
            path = nx.astar_path(G, start_node, goal_node, weight='weight')
            return np.array(path)
        except nx.NetworkXNoPath:
            print("No path found!")
            return np.array([self.start, self.start])

# =========================================================
# ITERATION 1: No Wind Field (straight-line baseline)
# =========================================================
print("=== ITERATION 1: No Wind ===")
planner_AB_nowind = EnergyPlanner3D(start=[-2.5, -2.5, 1.0], goal=[2.5, 2.5, 1.0], v_cruise=1.0)
path_AB_nowind = planner_AB_nowind.find_path()

planner_BA_nowind = EnergyPlanner3D(start=[2.5, 2.5, 1.0], goal=[-2.5, -2.5, 1.0], v_cruise=1.0)
path_BA_nowind = planner_BA_nowind.find_path()

# =========================================================
# ITERATION 2: With Wind Fields
# =========================================================
print("\n=== ITERATION 2: With Wind ===")
wind = WindField()
wind.add_source(make_cylinder_source(
    center=[0, 0, 2], axis=[0, 1, 0], radius=1.0, speed=5.0   # Tunnel 1: +Y at Z=2
))
wind.add_source(make_cylinder_source(
    center=[0, 0, 4], axis=[-1, -1, 0], radius=1.0, speed=5.0  # Tunnel 2: -XY diagonal at Z=4
))

planner_AB_wind = EnergyPlanner3D(start=[-2.5, -2.5, 1.0], goal=[2.5, 2.5, 1.0], v_cruise=1.0, wind_field=wind)
path_AB_wind = planner_AB_wind.find_path()

planner_BA_wind = EnergyPlanner3D(start=[2.5, 2.5, 1.0], goal=[-2.5, -2.5, 1.0], v_cruise=1.0, wind_field=wind)
path_BA_wind = planner_BA_wind.find_path()

# =========================================================
# ITERATION 3: Wind Fields — Tunnel 2 reversed (+X,+Y)
# =========================================================
print("\n=== ITERATION 3: Tunnel 2 Reversed ===")
wind_rev = WindField()
wind_rev.add_source(make_cylinder_source(
    center=[0, 0, 2], axis=[0, 1, 0], radius=1.0, speed=5.0   # Tunnel 1: same as before
))
wind_rev.add_source(make_cylinder_source(
    center=[0, 0, 4], axis=[1, 1, 0], radius=1.0, speed=5.0   # Tunnel 2: REVERSED (+X,+Y)
))

planner_AB_rev = EnergyPlanner3D(start=[-2.5, -2.5, 1.0], goal=[2.5, 2.5, 1.0], v_cruise=1.0, wind_field=wind_rev)
path_AB_rev = planner_AB_rev.find_path()

planner_BA_rev = EnergyPlanner3D(start=[2.5, 2.5, 1.0], goal=[-2.5, -2.5, 1.0], v_cruise=1.0, wind_field=wind_rev)
path_BA_rev = planner_BA_rev.find_path()


# =========================================================
# VISUALIZATION HELPERS
# =========================================================
def draw_wind_tunnels(ax, t2_direction=(-1, -1, 0)):
    """Draw the tunnel wireframes and wind quiver arrows.
    
    Args:
        t2_direction: Direction vector for tunnel 2 (default: -X,-Y).
    """
    # Tunnel 1 wireframe (+Y at X=0, Z=2)
    u_t = np.linspace(0, 2 * np.pi, 20)
    h_t = np.linspace(-3, 3, 20)
    U, H = np.meshgrid(u_t, h_t)
    ax.plot_wireframe(1.0 * np.cos(U), H, 1.0 * np.sin(U) + 2, color='cyan', alpha=0.1)

    # Tunnel 2 wireframe at Z=4
    t2_axis = np.array(t2_direction, dtype=float); t2_axis /= np.linalg.norm(t2_axis)
    t2_center = np.array([0.0, 0.0, 4.0])
    perp1 = np.cross(t2_axis, np.array([0, 0, 1])); perp1 /= np.linalg.norm(perp1)
    perp2 = np.cross(t2_axis, perp1); perp2 /= np.linalg.norm(perp2)
    theta_c, t_c = np.meshgrid(np.linspace(0, 2*np.pi, 20), np.linspace(-6, 6, 20))
    X2 = t2_center[0] + t_c*t2_axis[0] + 1.0*(np.cos(theta_c)*perp1[0] + np.sin(theta_c)*perp2[0])
    Y2 = t2_center[1] + t_c*t2_axis[1] + 1.0*(np.cos(theta_c)*perp1[1] + np.sin(theta_c)*perp2[1])
    Z2 = t2_center[2] + t_c*t2_axis[2] + 1.0*(np.cos(theta_c)*perp1[2] + np.sin(theta_c)*perp2[2])
    ax.plot_wireframe(X2, Y2, Z2, color='magenta', alpha=0.1)

    # Tunnel 1 quiver arrows
    xv, yv, zv = np.meshgrid(np.linspace(-0.8, 0.8, 3), np.linspace(-5, 5, 6), np.linspace(1.2, 2.8, 3))
    x_arr, y_arr, z_arr, u_w, v_w, w_w = [], [], [], [], [], []
    for i in range(xv.shape[0]):
        for j in range(xv.shape[1]):
            for k in range(xv.shape[2]):
                x, y, z = xv[i,j,k], yv[i,j,k], zv[i,j,k]
                if np.sqrt(x**2 + (z-2)**2) < 1.0:
                    x_arr.append(x); y_arr.append(y); z_arr.append(z)
                    u_w.append(0); v_w.append(1); w_w.append(0)
    ax.quiver(x_arr, y_arr, z_arr, u_w, v_w, w_w, length=1.0, color='cyan', alpha=0.5, normalize=True, label='Wind Tunnel 1 (+Y)')

    # Tunnel 2 quiver arrows
    t2_dir = t2_axis.copy()
    xv2, yv2, zv2 = [], [], []
    for t_val in np.linspace(-4, 4, 8):
        pt = t2_center + t_val * t2_dir
        xv2.append(pt[0]); yv2.append(pt[1]); zv2.append(pt[2])
    ax.quiver(xv2, yv2, zv2, [t2_dir[0]]*len(xv2), [t2_dir[1]]*len(xv2), [t2_dir[2]]*len(xv2),
              length=1.0, color='magenta', alpha=0.5, normalize=True, label='Wind Tunnel 2 (-XY diagonal)')


def format_ax(ax, title):
    """Apply common formatting to a 3D axis."""
    ax.set_xlabel('X (Side)')
    ax.set_ylabel('Y (Distance)')
    ax.set_zlabel('Z (Altitude)')
    ax.set_title(title)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(-1, 11)
    ax.set_box_aspect([1, 1, 1])


# =========================================================
# PLOTTING — Side by side comparison
# =========================================================
A = [-2.5, -2.5, 1.0]
B = [2.5, 2.5, 1.0]

fig = plt.figure(figsize=(26, 9))

# --- Left: No Wind ---
ax1 = fig.add_subplot(131, projection='3d')
ax1.plot(path_AB_nowind[:,0], path_AB_nowind[:,1], path_AB_nowind[:,2], 'g--', linewidth=3, label='A → B')
ax1.plot(path_BA_nowind[:,0], path_BA_nowind[:,1], path_BA_nowind[:,2], 'r-',  linewidth=3, label='B → A')
ax1.scatter(*A, c='blue',   s=100, label='Point A')
ax1.scatter(*B, c='orange', s=100, label='Point B')
format_ax(ax1, 'No Wind (Baseline)')

# --- Middle: With Wind (Tunnel 2: -X,-Y) ---
ax2 = fig.add_subplot(132, projection='3d')
ax2.plot(path_AB_wind[:,0], path_AB_wind[:,1], path_AB_wind[:,2], 'g--', linewidth=3, label='A → B')
ax2.plot(path_BA_wind[:,0], path_BA_wind[:,1], path_BA_wind[:,2], 'r-',  linewidth=3, label='B → A')
ax2.scatter(*A, c='blue',   s=100, label='Point A')
ax2.scatter(*B, c='orange', s=100, label='Point B')
draw_wind_tunnels(ax2, t2_direction=(-1, -1, 0))
format_ax(ax2, 'Wind (Tunnel 2: -X,-Y)')

# --- Right: Tunnel 2 Reversed (+X,+Y) ---
ax3 = fig.add_subplot(133, projection='3d')
ax3.plot(path_AB_rev[:,0], path_AB_rev[:,1], path_AB_rev[:,2], 'g--', linewidth=3, label='A → B')
ax3.plot(path_BA_rev[:,0], path_BA_rev[:,1], path_BA_rev[:,2], 'r-',  linewidth=3, label='B → A')
ax3.scatter(*A, c='blue',   s=100, label='Point A')
ax3.scatter(*B, c='orange', s=100, label='Point B')
draw_wind_tunnels(ax3, t2_direction=(1, 1, 0))
format_ax(ax3, 'Wind (Tunnel 2: +X,+Y reversed)')

plt.tight_layout()
plt.show()