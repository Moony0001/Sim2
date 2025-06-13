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