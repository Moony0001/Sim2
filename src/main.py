import pygame
from simulation import Simulation

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Traffic Simulator")
    clock = pygame.time.Clock()
    sim = Simulation(screen)

    running = True
    while running: 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        sim.update()
        sim.draw()
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()

if __name__ == "__main__":
    main()