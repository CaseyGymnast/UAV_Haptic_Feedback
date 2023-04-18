#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist

import numpy as np
import pygame
  
# Initialize pygame
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

RADIUS = True
COLOR = GREEN


class Drone(Node):
    def __init__(self, pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0):
        super.__init__('drone')

        # Class Variables
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.vel_x = 0
        self.vel_y = 0
        self.disturbance_x = disturbance_x
        self.disturbance_y = disturbance_y     
        self.drone = pygame.draw.circle(screen, COLOR, (self.pos_x, self.pos_y), RADIUS)

        # Publishers
        self.pub_pose = self.create_publisher(Point, '/drone_pose', 1)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.callback_twist, 1)
  
    def callback_twist(self, msg):
        """Updates velocities based on /cmd_vel topic"""

        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y

    def display(self):
        self.drone = pygame.draw.circle(screen, COLOR, (self.pos_x, self.pos_y), RADIUS)
  
    def update(self):
        # Update Positions based on Velocities and Disturbances
        self.pos_x += self.vel_x + self.disturbance_x * (0.75 - 2*np.random.random())
        self.pos_y += self.vel_y + self.disturbance_y * (0.75 - 2*np.random.random())
        
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
    drone = Drone(pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=4, radius=7, color=WHITE)
  
    x_displacement = 0
    y_displacement = 0

    while running:
        screen.fill(BLACK)
  
        # Quit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
  
        # Updating the objects
        drone.update()
  
        # Displaying the objects on the screen
        drone.display()
  
        pygame.display.update()
        clock.tick(FPS)  # Adjusting the frame rate

if __name__ == "__main__":
	main()
	pygame.quit()