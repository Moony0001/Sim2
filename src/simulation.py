from road import Road
from vehicle import Vehicle
from obstacle import Obstacle

class Simulation:
    def __init__(self, screen):
        self.screen = screen
        self.roads = []
        self.vehicles = []
        self._setup()
    
    def _setup(self):
        road = Road((100, 300), (700, 300), width=100)
        self.roads.append(road)
        vehicle = Vehicle(road)
        self.vehicles.append(vehicle)
        self.obstacles = [
            Obstacle((400, 300), 12),
            Obstacle((550, 300), 10)
        ]

    def update(self):
        for vehicle in self.vehicles:
            vehicle.update(self.vehicles, self.roads[0], self.obstacles)
    
    def draw(self):
        self.screen.fill((30, 30, 30))
        for road in self.roads:
            road.draw(self.screen)
        for vehicle in self.vehicles:
            vehicle.draw(self.screen)
        for obstacle in self.obstacles:
            obstacle.draw(self.screen)
    
    def get_state(self):
        # Return a dict of all relevant simulation state
        return {
            "vehicles": [
                {
                    "position": v.position[:],
                    "velocity": v.velocity[:],
                    "acceleration": v.acceleration[:],
                    "angle": v.angle,
                    "smoothed_force": v.smoothed_force[:],
                }
                for v in self.vehicles
            ],
            "obstacles": [
                {
                    "pos": o.pos[:],
                    "radius": o.radius
                }
                for o in self.obstacles
            ],
            # Add more if you have other stateful objects
        }

    def set_state(self, state):
        # Restore simulation state from a dict
        for v, v_state in zip(self.vehicles, state["vehicles"]):
            v.position = v_state["position"][:]
            v.velocity = v_state["velocity"][:]
            v.acceleration = v_state["acceleration"][:]
            v.angle = v_state["angle"]
            v.smoothed_force = v_state["smoothed_force"][:]
        for o, o_state in zip(self.obstacles, state["obstacles"]):
            o.pos = o_state["pos"][:]
            o.radius = o_state["radius"]
        # Add more if you have other stateful objects