import argparse
import ast
import random
import subprocess
import sys

### HYPERPARAMETERS ###
BACKGROUND_COLOR = (0, 0, 0)  # Default black
SQUARE_COLOR = (255, 0, 247)  # Default red
FPS = 60  # Frames per second
CIRCLE_RADIUS_RANGE = (25, 150)  # Default radius range
DISPLAY_DIMENSIONS = (800, 600)  # Default display dimensions


### /HYPERPARAMETERS ###

def parse_flags():
    parser = argparse.ArgumentParser(description="Pygame display configuration flags")

    parser.add_argument("-s", "--square_size", type=str, default=100, help="Size of each square")

    parser.add_argument("-n", "--number_squares", type=str, default=3,
                        help="Number of squares in each dimension (total number of squares will be n x n)")

    parser.add_argument("-c", "--color", type=ast.literal_eval, default=(255, 0, 247),
                        help="Shape color as (R, G, B). Default: (255, 255, 255)")

    parser.add_argument("-b", "--background", type=ast.literal_eval, default=(0, 0, 0),
                        help="Background color as (R, G, B). Default: (255, 255, 255)")

    args = parser.parse_args()

    print(args)

    return args


def main(args):
    # Update global variables based on parsed flags
    BACKGROUND_COLOR = args.background
    SQUARE_COLOR = args.color
    WINDOW_SIDE_X = 1440
    WINDOW_SIDE_Y = 880
    NUM_SQUARES = 6

    NUM_X = 4
    NUM_Y = 2

    SQUARE_SIDE_X = WINDOW_SIDE_X / NUM_X
    SQUARE_SIDE_Y = WINDOW_SIDE_Y / NUM_Y

    # Initialize Pygame
    pg.init()
    screen = pg.display.set_mode((WINDOW_SIDE_X, WINDOW_SIDE_Y))
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

                    x = random.randint(0, NUM_X - 1)
                    y = random.randint(0, NUM_Y - 1)
                    col = random.randint(0, 1)
                    if col == 0:
                        pg.draw.rect(screen, SQUARE_COLOR,
                                 (x * SQUARE_SIDE_X, y * SQUARE_SIDE_Y, SQUARE_SIDE_X, SQUARE_SIDE_Y))
                    else:
                        pg.draw.rect(screen, (247, 255, 0),
                                     (x * SQUARE_SIDE_X, y * SQUARE_SIDE_Y, SQUARE_SIDE_X, SQUARE_SIDE_Y))
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
