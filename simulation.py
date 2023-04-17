#!/usr/bin/env python3

import numpy as np
import pygame
import serial
import time
import json

pygame.init()

# RGB values of standard colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
  
# Basic parameters of the SCREEN
WIDTH, HEIGHT = 900, 600
FONT = pygame.font.Font('freesansbold.ttf', 20)
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
CLOCK = pygame.time.Clock()
pygame.display.set_caption("UAV Flight")
FPS = 30  # Used to adjust the frame rate
CONTROL_VELOCITY = 4  # Pixels per Frame
INTERACTION_RESET = 100
MAX_DISTANCE = np.sqrt(WIDTH**2 + HEIGHT**2)
MAX_RTM = 140


class Drone:
    def __init__(self, pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0, color=RED, radius=7):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.disturbance_x = disturbance_x
        self.disturbance_y = disturbance_y   
        self.color = color
        self.radius = radius
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def display(self):
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def update(self, displacement_x, displacement_y):
        # Update Positions based on Input and Disturbances
        self.pos_x += displacement_x + self.disturbance_x * (0.75 - 2*np.random.random())
        self.pos_y += displacement_y + self.disturbance_y * (0.75 - 2*np.random.random())
        
        # Confine to Window
        self.pos_x = max(min(WIDTH, self.pos_x), 0)
        self.pos_y = max(min(HEIGHT, self.pos_y), 0)
  
    def reset(self):
        self.pos_x = WIDTH//2
        self.pos_y = HEIGHT//2
        
    def getRect(self):
        return self.draw


class Waypoint:
    def __init__(self, pos_x=WIDTH//2, pos_y=HEIGHT//2, color=WHITE, radius=50):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.color = color
        self.radius = radius
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)

        self.interaction_count = 0
        self.update()
  
    def display(self):
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def interaction(self, drone_x, drone_y):
        interacting = np.sqrt((drone_x - self.pos_x)**2 + (drone_y - self.pos_y)**2) <= self.radius
        if interacting:
            self.interaction_count += 1
            self.color = GREEN
        else:
            self.interaction_count = 0
            self.color = WHITE

        if self.interaction_count > INTERACTION_RESET:
            self.interaction_count = 0
            self.update()
            self.interaction(drone_x, drone_y)

    def update(self):
        # Randomly Select Waypoint
        self.pos_x = np.random.random_sample() * WIDTH
        self.pos_y = np.random.random_sample() * HEIGHT
        
        # Confine to Window
        self.pos_x = max(min(WIDTH, self.pos_x), 0)
        self.pos_y = max(min(HEIGHT, self.pos_y), 0)
  
    def reset(self):
        self.pos_x = WIDTH//2
        self.pos_y = HEIGHT//2
        
    def getRect(self):
        return self.draw
    
class Teensy:
    def __init__(self):
        self.port = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
        self.diff_x = 0
        self.diff_y = 0
        self.distance = 0
        self.serial_count = 0
        self.magnitudes = {'up':'0', 'right':'0', 'down':'0', 'left':'0'}
    
    def evaluate(self, drone, waypoint):
        self.diff_x = drone.pos_x - waypoint.pos_x
        self.diff_y = drone.pos_y - waypoint.pos_y
        self.distance = np.sqrt(self.diff_x**2 + self.diff_y**2)
        self.interacting = self.distance <= waypoint.radius

        self.update_drv_magnitudes()

    def update_drv_magnitudes(self):
        if self.diff_x > 0:
            self.magnitudes['left'] = str(self.diff_x/MAX_DISTANCE * (MAX_RTM - 30) + 30)
            self.magnitudes['right'] = str(0)
        else:
            self.magnitudes['right'] = str(-self.diff_x/MAX_DISTANCE * (MAX_RTM - 30) + 30)
            self.magnitudes['left'] = str(0)

        if self.diff_y > 0:
            self.magnitudes['up'] = str(self.diff_y/MAX_DISTANCE * (MAX_RTM - 30) + 30)
            self.magnitudes['down'] = str(0)
        else:
            self.magnitudes['down'] = str(-self.diff_y/MAX_DISTANCE * (MAX_RTM - 30) + 30)
            self.magnitudes['up'] = str(0)

    def send_serial(self):
        data = json.dumps(self.magnitudes)
    
        if self.port.isOpen():
            self.port.write(data.encode('ascii'))
            self.port.flush()
            try:
                incoming = self.port.readline().decode("utf-8")
            except Exception as e:
                print(e)
                pass
        else:
            print("opening error")


def main():
    running = True
  
    # Define objects
    drone = Drone(pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0)
    waypoint = Waypoint()
    teensy = Teensy()
  
    x_displacement = 0
    y_displacement = 0

    while running:
        SCREEN.fill(BLACK)

        # Handle User Input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    y_displacement = -CONTROL_VELOCITY
                if event.key == pygame.K_DOWN:
                    y_displacement = CONTROL_VELOCITY
                if event.key == pygame.K_LEFT:
                    x_displacement = -CONTROL_VELOCITY
                if event.key == pygame.K_RIGHT:
                    x_displacement = CONTROL_VELOCITY
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    y_displacement = 0
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    x_displacement = 0
  
        # Update Drone Position
        drone.update(x_displacement, y_displacement)

        # Handle Interaction
        waypoint.interaction(drone.pos_x, drone.pos_y)

        # Communicate with Serial
        if teensy.serial_count > 5:
            teensy.serial_count = 0
            teensy.evaluate(drone, waypoint)
            teensy.send_serial()

        # Display objects on the SCREEN
        waypoint.display()
        drone.display()

        teensy.serial_count += 1
  
        pygame.display.update()
        CLOCK.tick(FPS)  # Adjusting the frame rate

if __name__ == "__main__":
	main()
	pygame.quit()
        