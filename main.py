import env,sensors
import pygame
import math

environment = env.buildEnvironment((600, 1200))
running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.update()

