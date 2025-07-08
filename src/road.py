import pygame

class Road:
    def __init__(self, start_pos, end_pos, width=47, color=(100, 100, 100)):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.width = width
        self.color = color

    def draw(self, screen):
        x1, y1 = self.start_pos
        x2, y2 = self.end_pos
        dx, dy = x2 - x1, y2 - y1
        length = (dx**2 + dy**2) ** 0.5
        angle = pygame.math.Vector2(dx, dy).angle_to((1, 0))
        road_rect = pygame.Surface((length, self.width))
        road_rect.fill(self.color)
        rotated = pygame.transform.rotate(road_rect, -angle)
        rect = rotated.get_rect(center=((x1 + x2) // 2, (y1 + y2) // 2))
        screen.blit(rotated, rect)

    def get_walls(self):
        x1, y1 = self.start_pos
        x2, y2 = self.end_pos
        dx, dy = x2 - x1, y2 - y1
        length = (dx**2 + dy**2) ** 0.5
        nx, ny = -dy / length, dx / length  # Normal vector
        half_w = self.width / 2
        wall1_start = (x1 + nx * half_w, y1 + ny * half_w)
        wall1_end = (x2 + nx * half_w, y2 + ny * half_w)
        wall2_start = (x1 - nx * half_w, y1 - ny * half_w)
        wall2_end = (x2 - nx * half_w, y2 - ny * half_w)
        return [(wall1_start, wall1_end), (wall2_start, wall2_end)]