import random
import subprocess
import sys

### HYPERPARAMETERS ###
BACKGROUND_COLOR = (0, 0, 0) # Black
SQUARE_COLOR = (255, 0, 0) # White

FPS = 60 # Frames per second

CIRCLE_RADIUS_RANGE = (25, 150) # Radius of each circle uniformally randomly selected from this range
DISPLAY_DIMENSIONS = (800, 600)
### /HYPERPARAMETERS ###

class Circle:
    def __init__(self):
        self.radius = self.get_radius()
        self.center = self.get_center()
        self.color = self.get_color()
    
    def get_radius(self):
        return random.uniform(CIRCLE_RADIUS_RANGE[0], CIRCLE_RADIUS_RANGE[1])

    def get_center(self):
        x = random.uniform(self.radius, DISPLAY_DIMENSIONS[0] - self.radius)
        y = random.uniform(self.radius, DISPLAY_DIMENSIONS[1] - self.radius)
        return (x, y)
    
    def get_color(self):
        return SQUARE_COLOR

def main():
    # Initialize all parameters
    pg.init()
    screen = pg.display.set_mode(DISPLAY_DIMENSIONS)
    pg.display.set_caption("EE106A Final Project Display")
    clock = pg.time.Clock()

    running = True
    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False 
            elif event.type == pg.KEYDOWN:
                print(f"Key {event.key} down!")
                if event.key == pg.K_SPACE:
                    print("Space pressed.")

                    # Draw everything
                    screen.fill(BACKGROUND_COLOR)

                    circle = Circle()
                    pg.draw.circle(screen, circle.color, circle.center, circle.radius)
                    del circle

                    # Pygame updates
                    pg.display.flip()
                    clock.tick(FPS)
    
    pg.quit()
    return
    

if __name__ == '__main__':
    # Install pygame
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pygame'])
    import pygame as pg
    main()
