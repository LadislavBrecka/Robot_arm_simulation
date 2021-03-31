import pygame
import sys
from pygame_robot_class import RobotArm
from Colors import *


def run():
    # End condition
    game_loop = True

    # Setting up screen
    SIZE_X, SIZE_Y = 1000, 500
    pygame.init()
    screen = pygame.display.set_mode((SIZE_X, SIZE_Y))
    my_font1 = pygame.font.SysFont("monospace", 30)

    # Start values
    x, y = 200, 200
    robot = RobotArm()
    clock = pygame.time.Clock()

    # Game Loop
    while game_loop:
        clock.tick(15)
        screen.fill(white)

        time = pygame.time.get_ticks()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        robot.draw(screen)

        pygame.display.update()

