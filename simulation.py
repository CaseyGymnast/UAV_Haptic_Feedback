#!/usr/bin/env python3

import numpy as np
import pygame
  
pygame.init()
  
# RGB values of standard colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
  
# Basic parameters of the screen
font20 = pygame.font.Font('freesansbold.ttf', 20)
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("UAV Flight")
clock = pygame.time.Clock()
FPS = 30  # Used to adjust the frame rate
CONTROL_VELOCITY = 4  # Pixels per Frame


class Drone:
    def __init__(self, pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0, radius=True, color=GREEN):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.disturbance_x = disturbance_x
        self.disturbance_y = disturbance_y
        self.radius = radius
        self.color = color        
        self.drone = pygame.draw.circle(screen, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def display(self):
        self.drone = pygame.draw.circle(screen, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def update(self, displacement_x, displacement_y):
        # Update Positions based on Input and Disturbances
        self.pos_x += displacement_x + self.disturbance_x * (1 - 2*np.random.random())
        self.pos_y += displacement_y + self.disturbance_y * (1 - 2*np.random.random())
        
        # Confine to Window
        self.pos_x = max(min(WIDTH, self.pos_x), 0)
        self.pos_y = max(min(HEIGHT, self.pos_y), 0)
  
    def reset(self):
        self.pos_x = WIDTH//2
        self.pos_y = HEIGHT//2
        
    def getRect(self):
        return self.drone


def main():
    running = True
  
    # Defining the objects
    drone = Drone(pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0, radius=7, color=WHITE)
  
    while running:
        screen.fill(BLACK)
  
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    y_displacement = CONTROL_VELOCITY
                if event.key == pygame.K_DOWN:
                    y_displacement = -CONTROL_VELOCITY
                if event.key == pygame.K_LEFT:
                    x_displacement = -CONTROL_VELOCITY
                if event.key == pygame.K_RIGHT:
                    x_displacement = CONTROL_VELOCITY
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    y_displacement = 0
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    x_displacement = 0
  
        # Updating the objects
        drone.update(x_displacement, y_displacement)
  
        # Displaying the objects on the screen
        drone.display()
  
        pygame.display.update()
        clock.tick(FPS)  # Adjusting the frame rate
