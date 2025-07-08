import pygame
import copy
from simulation import Simulation

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Traffic Simulator")
    clock = pygame.time.Clock()
    sim = Simulation(screen)

    running = True
    paused = False
    frame = 0
    states = []

    # Save the initial state
    states.append(copy.deepcopy(sim.get_state()))

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:  # Pause/Play toggle
                    paused = not paused
                elif event.key == pygame.K_r:    # Replay
                    if states:
                        frame = 0
                        sim.set_state(copy.deepcopy(states[frame]))
                        paused = True
                elif event.key == pygame.K_RIGHT:  # Step forward
                    if paused:
                        if frame < len(states) - 1:
                            frame += 1
                            sim.set_state(copy.deepcopy(states[frame]))
                        else:
                            # If at the last frame, run one more simulation step and save it
                            sim.update()
                            states.append(copy.deepcopy(sim.get_state()))
                            frame += 1
                            sim.set_state(copy.deepcopy(states[frame]))
                elif event.key == pygame.K_LEFT:   # Step backward
                    if paused and frame > 0:
                        frame -= 1
                        sim.set_state(copy.deepcopy(states[frame]))

        if not paused:
            sim.update()
            # Save a deep copy of the current state only if it's a new frame
            if frame == len(states) - 1:
                states.append(copy.deepcopy(sim.get_state()))
                frame += 1
            else:
                # If user played after stepping back, discard "future" states and continue
                states = states[:frame+1]
                states.append(copy.deepcopy(sim.get_state()))
                frame += 1

        screen.fill((255, 255, 255))
        sim.draw()
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()