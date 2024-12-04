import argparse
import ast
import random
import subprocess
import sys

### HYPERPARAMETERS ###
BACKGROUND_COLOR = (0, 0, 0)  # Default black
SQUARE_COLOR = (255, 0, 0)  # Default red
FPS = 60  # Frames per second
CIRCLE_RADIUS_RANGE = (25, 150)  # Default radius range
DISPLAY_DIMENSIONS = (800, 600)  # Default display dimensions
### /HYPERPARAMETERS ###

class Circle:
    def __init__(self, radius_range, color, display_dims):
        self.radius = self.get_radius(radius_range)
        self.center = self.get_center(display_dims)
        self.color = color

    def get_radius(self, radius_range):
        return random.uniform(radius_range[0], radius_range[1])

    def get_center(self, display_dims):
        x = random.uniform(self.radius, display_dims[0] - self.radius)
        y = random.uniform(self.radius, display_dims[1] - self.radius)
        return (x, y)

def parse_flags():
    parser = argparse.ArgumentParser(description="Pygame display configuration flags")

    # Shape flag
    parser.add_argument("-s", "--shape", choices=["square", "circle"], default="square",
                        help="Shape to display: square or circle (default: square)")

    # Radius or side length
    parser.add_argument("-r", "--size", type=str, default=None,
                        help="Radius or side length (e.g., '50' or '50,100' for a range). Default: 10% of average display dimension")

    # Display dimensions
    parser.add_argument("-d", "--display", type=ast.literal_eval, default=(800, 600),
                        help="Display dimensions in the format (width, height). Default: (800, 600)")

    # Background color
    parser.add_argument("-b", "--background", type=ast.literal_eval, default=(0, 0, 0),
                        help="Background color as (R, G, B). Default: (0, 0, 0)")

    # Shape color
    parser.add_argument("-c", "--color", type=ast.literal_eval, default=(255, 255, 255),
                        help="Shape color as (R, G, B). Default: (255, 255, 255)")

    args = parser.parse_args()

    # Handle the radius as a range if specified
    if args.size:
        try:
            size_values = [int(x) for x in args.size.split(",")]
            if len(size_values) == 1:
                args.size = (size_values[0], size_values[0])
            elif len(size_values) == 2:
                args.size = tuple(size_values)
            else:
                raise ValueError("Invalid radius format.")
        except ValueError:
            raise ValueError("Invalid radius format. Must be a single number or a range, e.g., '50' or '50,100'.")
    else:
        avg_dim = (args.display[0] + args.display[1]) / 2
        default_radius = int(avg_dim * 0.1)
        args.size = (default_radius, default_radius)

    return args

def main(args):
    global BACKGROUND_COLOR, SQUARE_COLOR, DISPLAY_DIMENSIONS, CIRCLE_RADIUS_RANGE

    # Update global variables based on parsed flags
    BACKGROUND_COLOR = args.background
    SQUARE_COLOR = args.color
    DISPLAY_DIMENSIONS = args.display
    CIRCLE_RADIUS_RANGE = args.size

    # Initialize Pygame
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
                if event.key == pg.K_q:
                    pg.quit()
                    return
                if event.key == pg.K_SPACE:
                    print("Space pressed.")

                    # Draw everything
                    screen.fill(BACKGROUND_COLOR)

                    if args.shape == "circle":
                        circle = Circle(CIRCLE_RADIUS_RANGE, SQUARE_COLOR, DISPLAY_DIMENSIONS)
                        pg.draw.circle(screen, circle.color, circle.center, int(circle.radius))
                    elif args.shape == "square":
                        side = random.uniform(*CIRCLE_RADIUS_RANGE)
                        x = random.uniform(0, DISPLAY_DIMENSIONS[0] - side)
                        y = random.uniform(0, DISPLAY_DIMENSIONS[1] - side)
                        pg.draw.rect(screen, SQUARE_COLOR, (x, y, side, side))

                    # Pygame updates
                    pg.display.flip()
                    clock.tick(FPS)

    pg.quit()

if __name__ == '__main__':
    # Ensure Pygame is installed
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pygame'])
    import pygame as pg

    # Parse input
    args = parse_flags()
    print(f"Got flags: {args}")

    main(args)
