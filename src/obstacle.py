import pygame

class Obstacle:
    def __init__(self, pos, radius=12, color=(0, 0, 255)):
        self.pos = pos
        self.radius = radius
        self.color = color

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.pos[0]), int(self.pos[1])), self.radius)