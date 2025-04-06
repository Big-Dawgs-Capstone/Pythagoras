import numpy as np
import matplotlib.pyplot as plt

# Reusing simplified Pose class
class Pose:
    def __init__(self):
        self.position = type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.orientation = type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()

class WaypointNode:
    def __init__(self, pose):
        self.pose = pose
        self.next = None
        self.head = False

class SimulatedAutonomousFixed:
    def __init__(self):
        self.transformation_mat = np.array([
            [0.99756421, 0.04327479, -0.05470779],
            [-0.04392193, 0.99897786, -0.01068201],
            [0.05411896, 0.01305886, 0.99844527]
        ])
        self.orangepose = self.random_pose()
        self.center_pose = self.random_pose(center=True)
        self.object_dims = np.array([0.5, 0.5, 1.5])
        self.secondary_target = None
        self.path_taken = []

    def random_pose(self, center=False):
        pose = Pose()
        if center:
            pose.position.x = np.random.uniform(0, 2)
            pose.position.y = np.random.uniform(0, 2)
        else:
            pose.position.x = np.random.uniform(-2, 0)
            pose.position.y = np.random.uniform(-2, 0)
        pose.position.z = 1.0
        return pose

    def transform_pose(self, pose: Pose) -> Pose:
        raw_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        trans_position = self.transformation_mat @ raw_position
        new_pose = Pose()
        new_pose.position.x, new_pose.position.y, new_pose.position.z = trans_position
        return new_pose

    def ray_cyl_intersect(self, direction, pose, cyl_cent, cyl_rad):
        dx, dy = direction[0], direction[1]
        ox, oy = pose[0] - cyl_cent[0], pose[1] - cyl_cent[1]
        a = dx**2 + dy**2
        b = 2 * (ox*dx + oy*dy)
        c = ox**2 + oy**2 - cyl_rad**2
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return None
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        t = min(t1, t2)
        if t < 0:
            return None
        return pose + t * direction

    def autonomous_phase_two(self):
        center = self.transform_pose(self.center_pose)
        self.object_center = center
        cyl_cent = np.array([center.position.x, center.position.y, center.position.z])
        pose = np.array([self.orangepose.position.x, self.orangepose.position.y, self.orangepose.position.z])
        direction = (cyl_cent - pose) / np.linalg.norm(cyl_cent - pose)
        r = max(self.object_dims[0], self.object_dims[1]) + 1.0
        intersection_point = self.ray_cyl_intersect(direction, pose, cyl_cent, r)
        if intersection_point is None:
            raise RuntimeError("No intersection point found")
        self.secondary_target = Pose()
        self.secondary_target.position.x = intersection_point[0]
        self.secondary_target.position.y = intersection_point[1]
        self.secondary_target.position.z = self.center_pose.position.z
        self.path_taken.append(self.secondary_target)

    def generate_waypoints(self, num_points=30):
        cx = self.object_center.position.x
        cy = self.object_center.position.y
        sx = self.secondary_target.position.x
        sy = self.secondary_target.position.y
        z = self.secondary_target.position.z
        radius = np.linalg.norm([cx - sx, cy - sy])
        start_angle = np.arctan2(sy - cy, sx - cx)
        waypoints = []
        for i in range(num_points):
            angle = start_angle + 2 * np.pi * i / num_points
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            waypoints.append(pose)
        return waypoints

    def autonomous_phase_three(self, num_laps=2):
        waypoints = self.generate_waypoints()
        for _ in range(num_laps):
            self.path_taken.extend(waypoints)
        self.path_taken.append(self.secondary_target)

    def visualize_path(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        xs = [p.position.x for p in self.path_taken]
        ys = [p.position.y for p in self.path_taken]
        zs = [p.position.z for p in self.path_taken]
        ax.plot(xs, ys, zs, label="Trajectory", marker='o')
        ax.scatter(self.center_pose.position.x, self.center_pose.position.y, self.center_pose.position.z, c='r', label='Object Center')
        ax.scatter(self.orangepose.position.x, self.orangepose.position.y, self.orangepose.position.z, c='g', label='Start Pose')
        ax.legend()
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.title("Fixed Simulation Path Planning")
        plt.tight_layout()
        plt.show()

sim = SimulatedAutonomousFixed()
sim.autonomous_phase_two()
sim.autonomous_phase_three(num_laps=2)
sim.visualize_path()
