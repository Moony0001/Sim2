import pygame  
import math    
import numpy as np  

# Function to get the four corners of a rectangle (vehicle) in world coordinates
def get_corners(pos, angle_deg, length, width):
    """Returns the 4 corners of the rectangle (vehicle) in world coordinates."""
    angle = math.radians(angle_deg)  # Convert angle from degrees to radians
    dx = length / 2  # Half the length of the rectangle
    dy = width / 2   # Half the width of the rectangle
    # Rectangle corners relative to center (before rotation)
    corners = np.array([
        [ dx,  dy],   # Front-right corner
        [ dx, -dy],   # Front-left corner
        [-dx, -dy],   # Rear-left corner
        [-dx,  dy]    # Rear-right corner
    ])
    # Rotation matrix for rotating the rectangle
    rot = np.array([
        [math.cos(angle), -math.sin(angle)],  # First row of rotation matrix
        [math.sin(angle),  math.cos(angle)]   # Second row of rotation matrix
    ])
    rotated = corners @ rot.T  # Rotate all corners
    # Translate corners to world coordinates
    return [(pos[0] + p[0], pos[1] + p[1]) for p in rotated]

# Function to compute minimum distance between two rectangles given their corners
def rect_rect_min_dist(corners1, corners2):
    """Returns the minimum distance between two rectangles given their corners."""
    min_dist = float('inf')  # Initialize minimum distance as infinity
    # For each edge in rect1
    for i in range(4):
        a1 = np.array(corners1[i])  # Start point of edge in rect1
        a2 = np.array(corners1[(i+1)%4])  # End point of edge in rect1
        # For each corner in rect2
        for b in corners2:
            b = np.array(b)  # Convert corner to numpy array
            # Project b onto edge a1-a2
            ab = a2 - a1  # Vector along the edge
            t = np.dot(b - a1, ab) / (np.dot(ab, ab) + 1e-8)  # Projection scalar (avoid division by zero)
            t = np.clip(t, 0, 1)  # Clamp t to [0, 1] to stay on the edge
            closest = a1 + t * ab  # Closest point on edge to b
            dist = np.linalg.norm(b - closest)  # Euclidean distance from b to closest point
            min_dist = min(min_dist, dist)  # Update minimum distance if smaller
    # Repeat for edges in rect2 and corners in rect1
    for i in range(4):
        a1 = np.array(corners2[i])  # Start point of edge in rect2
        a2 = np.array(corners2[(i+1)%4])  # End point of edge in rect2
        for b in corners1:
            b = np.array(b)  # Convert corner to numpy array
            ab = a2 - a1  # Vector along the edge
            t = np.dot(b - a1, ab) / (np.dot(ab, ab) + 1e-8)  # Projection scalar
            t = np.clip(t, 0, 1)  # Clamp t to [0, 1]
            closest = a1 + t * ab  # Closest point on edge to b
            dist = np.linalg.norm(b - closest)  # Distance from b to closest point
            min_dist = min(min_dist, dist)  # Update minimum distance
    return min_dist  # Return the minimum distance found

# Function to compute minimum distance between a rectangle and a circle
def rect_circle_min_dist(rect_corners, circle_pos, circle_radius):
    """Returns the minimum distance between a rectangle and a circle."""
    min_dist = float('inf')  # Initialize minimum distance as infinity
    circle = np.array(circle_pos)  # Convert circle position to numpy array
    # For each edge of the rectangle
    for i in range(4):
        a1 = np.array(rect_corners[i])  # Start point of edge
        a2 = np.array(rect_corners[(i+1)%4])  # End point of edge
        ab = a2 - a1  # Vector along the edge
        t = np.dot(circle - a1, ab) / (np.dot(ab, ab) + 1e-8)  # Projection scalar
        t = np.clip(t, 0, 1)  # Clamp t to [0, 1]
        closest = a1 + t * ab  # Closest point on edge to circle center
        dist = np.linalg.norm(circle - closest)  # Distance from circle center to closest point
        min_dist = min(min_dist, dist)  # Update minimum distance
    # Subtract the circle's radius to get edge-to-edge distance
    return min_dist - circle_radius  # Return edge-to-edge distance

# Vehicle class representing a vehicle in the simulation
class Vehicle:
    def __init__(self, road, goal=None, color=(255, 0, 0), length=30, width=12, max_speed= 12):
        self.road = road  # Reference to the road object
        self.color = color  # Color of the vehicle
        self.length = length  # Length of the vehicle
        self.width = width    # Width of the vehicle
        self.max_speed = max_speed  # Maximum speed of the vehicle
        self.position = list(road.start_pos)  # Current position of the vehicle (as a list)
        self.velocity = [1.0, 0.0]  # Initial velocity vector
        self.acceleration = [0.0, 0.0]  # Initial acceleration vector
        self.goal = goal if goal else road.end_pos  # Goal position for the vehicle
        self.angle = 0  # Initial orientation angle (degrees)
        self.smoothed_force = [0.0, 0.0]  # For force smoothing

    # Compute the social force acting on the vehicle
    def compute_social_force(self, vehicles, road, obstacles):
        desired_speed = self.max_speed  # Desired speed for the vehicle
        relaxation_time = 0.5  # Time to reach desired velocity

        # Driving force towards the goal
        goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]  # Vector to goal
        goal_dist = math.hypot(*goal_vec)  # Distance to goal
        if goal_dist > 0:
            goal_dir = [goal_vec[0]/goal_dist, goal_vec[1]/goal_dist]  # Unit vector towards goal
        else:
            goal_dir = [0, 0]  # If already at goal, no direction
        desired_velocity = [goal_dir[0]*desired_speed, goal_dir[1]*desired_speed]  # Desired velocity vector
        driving_force = [(desired_velocity[0] - self.velocity[0])/relaxation_time,
                         (desired_velocity[1] - self.velocity[1])/relaxation_time]  # Driving force vector

        # Repulsive force from other vehicles
        repulsive_force = [0.0, 0.0]  # Initialize repulsive force
        my_corners = get_corners(self.position, self.angle, self.length, self.width)  # Get corners of this vehicle
        for other in vehicles:  # Loop over all vehicles
            if other is self:
                continue  # Skip self
            other_corners = get_corners(other.position, other.angle, other.length, other.width)  # Get corners of other vehicle
            dist = rect_rect_min_dist(my_corners, other_corners)  # Minimum distance between rectangles
            min_dist = 5  # Safety margin (tunable)
            if dist > 2 * min_dist:
                continue  # Ignore if too far
            # Direction from other to self (using centers)
            dx = self.position[0] - other.position[0]  # X difference
            dy = self.position[1] - other.position[1]  # Y difference
            heading = math.atan2(self.velocity[1], self.velocity[0])  # Heading angle of this vehicle
            direction = math.atan2(dy, dx)  # Direction from other to self
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)  # Angle difference
            modulation = max(0.1, math.cos(angle_diff))  # Modulation factor for force
            force_mag = 100 * math.exp(-dist / (0.5 * min_dist)) * modulation  # Magnitude of repulsive force
            norm = math.hypot(dx, dy) + 1e-8  # Normalization factor (avoid division by zero)
            repulsive_force[0] += force_mag * (dx / norm)  # Add x-component of force
            repulsive_force[1] += force_mag * (dy / norm)  # Add y-component of force

        # Repulsive force from road walls (with edge-to-edge logic)
        wall_force = [0.0, 0.0]  # Initialize wall force
        for wall_start, wall_end in road.get_walls():  # Loop over all walls
            px, py = self.position  # Vehicle position
            x1, y1 = wall_start  # Wall start point
            x2, y2 = wall_end    # Wall end point
            dx, dy = x2 - x1, y2 - y1  # Wall vector
            length = math.hypot(dx, dy)  # Wall length
            if length == 0:
                continue  # Skip degenerate wall
            t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (length ** 2)))  # Projection scalar
            closest = (x1 + t * dx, y1 + t * dy)  # Closest point on wall to vehicle
            dist = math.hypot(px - closest[0], py - closest[1])  # Distance to wall
            min_dist = (self.width / 2) + 5  # Safety margin from wall
            influence_radius = min_dist * 2  # Influence radius for wall force
            if dist < 1e-5 or dist > influence_radius:
                continue  # Ignore if too close or too far
            heading = math.atan2(self.velocity[1], self.velocity[0])  # Heading angle
            direction = math.atan2(py - closest[1], px - closest[0])  # Direction from wall to vehicle
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)  # Angle difference
            modulation = max(0.1, math.cos(angle_diff))  # Modulation factor
            force_mag = 200 * math.exp(-(dist - min_dist) / 8) * modulation  # Magnitude of wall force
            # Add a tangential component to help steer along the wall
            wall_dx = px - closest[0]  # X difference
            wall_dy = py - closest[1]  # Y difference
            if dist > 0:
                tangent = [-wall_dy / dist, wall_dx / dist]  # Tangent vector
            else:
                tangent = [0, 0]  # No tangent if at the wall
            goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]  # Vector to goal
            cross = goal_vec[0]*tangent[1] - goal_vec[1]*tangent[0]  # Cross product for direction
            if cross < 0:
                tangent = [-tangent[0], -tangent[1]]  # Flip tangent if needed
            radial_weight = 0.7  # Weight for radial component
            tangential_weight = 0.7  # Weight for tangential component
            wall_force[0] += force_mag * (radial_weight * (wall_dx / dist) + tangential_weight * tangent[0])  # X-component
            wall_force[1] += force_mag * (radial_weight * (wall_dy / dist) + tangential_weight * tangent[1])  # Y-component

        # Repulsive force from obstacles (rectangle-circle)
        obstacle_force = [0.0, 0.0]  # Initialize obstacle force
        for obs in obstacles:  # Loop over all obstacles
            dist = rect_circle_min_dist(my_corners, obs.pos, obs.radius)  # Min distance to obstacle
            min_dist = 5  # Safety margin
            influence_radius = min_dist * 2  # Influence radius
            if dist > influence_radius:
                continue  # Ignore if too far
            # Direction from obstacle to self (using centers)
            dx = self.position[0] - obs.pos[0]  # X difference
            dy = self.position[1] - obs.pos[1]  # Y difference
            heading = math.atan2(self.velocity[1], self.velocity[0])  # Heading angle
            direction = math.atan2(dy, dx)  # Direction from obstacle to vehicle
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)  # Angle difference
            modulation = max(0.1, math.cos(angle_diff))  # Modulation factor
            force_mag = 300 * math.exp(-(dist - min_dist) / (0.5 * obs.radius)) * modulation  # Magnitude of obstacle force
            norm = math.hypot(dx, dy) + 1e-8  # Normalization factor
            tangent = [-dy / norm, dx / norm]  # Tangent vector
            goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]  # Vector to goal
            cross = goal_vec[0]*tangent[1] - goal_vec[1]*tangent[0]  # Cross product
            if cross < 0:
                tangent = [-tangent[0], -tangent[1]]  # Flip tangent if needed
            radial_weight = 0.7  # Weight for radial component
            tangential_weight = 0.7  # Weight for tangential component
            obstacle_force[0] += force_mag * (radial_weight * (dx / norm) + tangential_weight * tangent[0])  # X-component
            obstacle_force[1] += force_mag * (radial_weight * (dy / norm) + tangential_weight * tangent[1])  # Y-component

        total_force = [
            driving_force[0] + repulsive_force[0] + wall_force[0] + obstacle_force[0],  # Total x-force
            driving_force[1] + repulsive_force[1] + wall_force[1] + obstacle_force[1]   # Total y-force
        ]
        return total_force  # Return the total force

    # Update the vehicle's state (position, velocity, angle)
    def update(self, vehicles, road, obstacles):
        raw_force = self.compute_social_force(vehicles, road, obstacles)  # Compute total force
        alpha = 0.2  # Smoothing factor for force
        self.smoothed_force[0] = (1 - alpha) * self.smoothed_force[0] + alpha * raw_force[0]
        self.smoothed_force[1] = (1 - alpha) * self.smoothed_force[1] + alpha * raw_force[1]
        F_x, F_y = self.smoothed_force
        dt = 0.1  # Time step for updates

        # Compute force magnitude
        F_mag = math.hypot(F_x, F_y)
        if F_mag > 1e-5:
            F_x /= F_mag
            F_y /= F_mag

        # Compute current and desired direction
        alpha = math.atan2(F_y, F_x)  # Desired direction (radians)
        theta = math.radians(self.angle)  # Current direction (radians)

        # Compute angular difference
        d_theta = (alpha - theta + math.pi) % (2 * math.pi) - math.pi  # Shortest signed angle

        # Define max angular velocity (radians per time step)
        omega_max = math.radians(2)  # ~2 degrees per update step

        # Apply angular velocity limiter
        omega = max(-omega_max, min(omega_max, d_theta))

        # Update vehicle angle gradually
        theta += omega
        self.angle = math.degrees(theta)

        print(f"Vehicle angle: {self.angle:.2f} degrees")

        # Move in the current direction
        speed = min(F_mag, self.max_speed)
        self.velocity = [math.cos(theta) * speed, math.sin(theta) * speed]
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt

        # F_max = self.max_speed  # Use max_speed as the max step per update

        # # If force magnitude exceeds max, scale it down
        # if F_mag > F_max:
        #     F_x = F_x * F_max / F_mag
        #     F_y = F_y * F_max / F_mag

        # # Update position directly using the force as a step
        # self.position[0] += F_x * dt
        # self.position[1] += F_y * dt

        # # Optionally, update angle to face the direction of movement
        # self.angle = math.degrees(math.atan2(F_y, F_x))

    # Draw the vehicle on the screen
    def draw(self, screen):
        rect = pygame.Rect(0, 0, self.length, self.width)  # Create a pygame rectangle
        rect.center = (int(self.position[0]), int(self.position[1]))  # Set rectangle center to vehicle position
        car_surf = pygame.Surface((self.length, self.width), pygame.SRCALPHA)  # Create a surface for the car
        pygame.draw.rect(car_surf, self.color, (0, 0, self.length, self.width))  # Draw the rectangle (car) on the surface
        rotated = pygame.transform.rotate(car_surf, -self.angle)  # Rotate the car surface by the vehicle's angle
        rot_rect = rotated.get_rect(center=rect.center)  # Get the rectangle for the rotated surface
        screen.blit(rotated, rot_rect)  # Draw the rotated car on the screen