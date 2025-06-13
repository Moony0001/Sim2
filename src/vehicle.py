import pygame
import math

class Vehicle:
    def __init__(self, road, goal=None, color=(255, 0, 0), length=30, width=15, max_speed=2):
        self.road = road
        self.color = color
        self.length = length
        self.width = width
        self.max_speed = max_speed
        self.position = list(road.start_pos)
        self.velocity = [1.0, 0.0]
        self.acceleration = [0.0, 0.0]
        self.goal = goal if goal else road.end_pos
        self.angle = 0

    def compute_social_force(self, vehicles, road, obstacles):
        desired_speed = self.max_speed
        relaxation_time = 0.5

        # Driving force towards the goal
        goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]
        goal_dist = math.hypot(*goal_vec)
        if goal_dist > 0:
            goal_dir = [goal_vec[0]/goal_dist, goal_vec[1]/goal_dist]
        else:
            goal_dir = [0, 0]
        desired_velocity = [goal_dir[0]*desired_speed, goal_dir[1]*desired_speed]
        driving_force = [(desired_velocity[0] - self.velocity[0])/relaxation_time,
                         (desired_velocity[1] - self.velocity[1])/relaxation_time]

        # Repulsive force from other vehicles
        repulsive_force = [0.0, 0.0]
        for other in vehicles:
            if other is self:
                continue
            dx = self.position[0] - other.position[0]
            dy = self.position[1] - other.position[1]
            dist = math.hypot(dx, dy)
            min_dist = (self.length + other.length) / 2
            if dist < 1e-5 or dist > 2 * min_dist:
                continue
            heading = math.atan2(self.velocity[1], self.velocity[0])
            direction = math.atan2(dy, dx)
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)
            modulation = max(0.1, math.cos(angle_diff))
            force_mag = 100 * math.exp(-dist / (0.5 * min_dist)) * modulation
            repulsive_force[0] += force_mag * (dx / dist)
            repulsive_force[1] += force_mag * (dy / dist)

        # Repulsive force from road walls (with edge-to-edge logic)
        wall_force = [0.0, 0.0]
        for wall_start, wall_end in road.get_walls():
            px, py = self.position
            x1, y1 = wall_start
            x2, y2 = wall_end
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (length ** 2)))
            closest = (x1 + t * dx, y1 + t * dy)
            dist = math.hypot(px - closest[0], py - closest[1])
            min_dist = (self.width / 2) + 5  # 5px safety margin from wall
            influence_radius = min_dist * 2
            if dist < 1e-5 or dist > influence_radius:
                continue
            heading = math.atan2(self.velocity[1], self.velocity[0])
            direction = math.atan2(py - closest[1], px - closest[0])
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)
            modulation = max(0.1, math.cos(angle_diff))
            force_mag = 200 * math.exp(-(dist - min_dist) / 8) * modulation
            # Add a tangential component to help steer along the wall
            wall_dx = px - closest[0]
            wall_dy = py - closest[1]
            if dist > 0:
                tangent = [-wall_dy / dist, wall_dx / dist]
            else:
                tangent = [0, 0]
            goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]
            cross = goal_vec[0]*tangent[1] - goal_vec[1]*tangent[0]
            if cross < 0:
                tangent = [-tangent[0], -tangent[1]]
            radial_weight = 0.7
            tangential_weight = 0.7
            wall_force[0] += force_mag * (radial_weight * (wall_dx / dist) + tangential_weight * tangent[0])
            wall_force[1] += force_mag * (radial_weight * (wall_dy / dist) + tangential_weight * tangent[1])

        # Repulsive force from obstacles (with lateral/tangential component)
        obstacle_force = [0.0, 0.0]
        for obs in obstacles:
            dx = self.position[0] - obs.pos[0]
            dy = self.position[1] - obs.pos[1]
            dist = math.hypot(dx, dy)
            min_dist = (self.length / 2) + obs.radius + 5  # 5px safety margin
            influence_radius = min_dist * 2  # Start steering earlier
            if dist < 1e-5 or dist > influence_radius:
                continue
            # Radial (direct) repulsion
            heading = math.atan2(self.velocity[1], self.velocity[0])
            direction = math.atan2(dy, dx)
            angle_diff = abs((heading - direction + math.pi) % (2 * math.pi) - math.pi)
            modulation = max(0.1, math.cos(angle_diff))
            # Stronger force as you get closer to min_dist
            force_mag = 300 * math.exp(-(dist - min_dist) / (0.5 * obs.radius)) * modulation
            tangent = [-dy / dist, dx / dist]
            goal_vec = [self.goal[0] - self.position[0], self.goal[1] - self.position[1]]
            cross = goal_vec[0]*tangent[1] - goal_vec[1]*tangent[0]
            if cross < 0:
                tangent = [-tangent[0], -tangent[1]]
            radial_weight = 0.7
            tangential_weight = 0.7
            obstacle_force[0] += force_mag * (radial_weight * (dx / dist) + tangential_weight * tangent[0])
            obstacle_force[1] += force_mag * (radial_weight * (dy / dist) + tangential_weight * tangent[1])

        total_force = [
            driving_force[0] + repulsive_force[0] + wall_force[0] + obstacle_force[0],
            driving_force[1] + repulsive_force[1] + wall_force[1] + obstacle_force[1]
        ]
        return total_force

    def update(self, vehicles, road, obstacles):
        force = self.compute_social_force(vehicles, road, obstacles)
        self.acceleration = force
        # Smoothing: blend new velocity with old
        new_velocity = [
            self.velocity[0] + self.acceleration[0] * 0.1,
            self.velocity[1] + self.acceleration[1] * 0.1
        ]
        speed = math.hypot(*new_velocity)
        if speed > self.max_speed:
            new_velocity[0] = new_velocity[0] / speed * self.max_speed
            new_velocity[1] = new_velocity[1] / speed * self.max_speed
        # Blend for smooth steering
        self.velocity[0] = 0.7 * self.velocity[0] + 0.3 * new_velocity[0]
        self.velocity[1] = 0.7 * self.velocity[1] + 0.3 * new_velocity[1]
        self.position[0] += self.velocity[0] * 0.1
        self.position[1] += self.velocity[1] * 0.1
        self.angle = math.degrees(math.atan2(self.velocity[1], self.velocity[0]))

    def draw(self, screen):
        rect = pygame.Rect(0, 0, self.length, self.width)
        rect.center = (int(self.position[0]), int(self.position[1]))
        car_surf = pygame.Surface((self.length, self.width), pygame.SRCALPHA)
        pygame.draw.rect(car_surf, self.color, (0, 0, self.length, self.width))
        rotated = pygame.transform.rotate(car_surf, -self.angle)
        rot_rect = rotated.get_rect(center=rect.center)
        screen.blit(rotated, rot_rect)