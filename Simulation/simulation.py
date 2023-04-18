#!/usr/bin/env python3

import numpy as np
import pygame
import serial
import time
import json
import csv

pygame.init()

# Main File Parameters
HAPTICS_ENABLED = False
VISUALS_ENABLED = True
DATA_COLLECTION_ENABLED = False
START_PAUSED = True
FRAME_LIMIT = 1000
GRAB_ID = False

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

# Game Parameters
CONTROL_VELOCITY = 4  # Pixels per Frame
INTERACTION_RESET = 100
MAX_DISTANCE = np.sqrt(WIDTH**2 + HEIGHT**2)
MAX_RTM = 140  # Maximum Vibration Magnitude

class Drone:
    def __init__(self, pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0, color=RED, radius=7):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.disturbance_x = disturbance_x
        self.disturbance_y = disturbance_y   
        self.displacement_x = 0
        self.displacement_y = 0
        self.color = color
        self.radius = radius
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)

        self.keys = {'up':False, 'down':False, 'right':False, 'left':False}
  
    def display(self):
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def update(self):
        # Update Positions based on Input and Disturbances
        self.pos_x += self.displacement_x + self.disturbance_x * (0.75 - 2*np.random.random())
        self.pos_y += self.displacement_y + self.disturbance_y * (0.75 - 2*np.random.random())
        
        # Confine to Window
        self.pos_x = max(min(WIDTH, self.pos_x), 0)
        self.pos_y = max(min(HEIGHT, self.pos_y), 0)

    def handle(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                self.displacement_y = -CONTROL_VELOCITY
                self.keys['up'] = True
            if event.key == pygame.K_DOWN:
                self.displacement_y = CONTROL_VELOCITY
                self.keys['down'] = True
            if event.key == pygame.K_LEFT:
                self.displacement_x = -CONTROL_VELOCITY
                self.keys['left'] = True
            if event.key == pygame.K_RIGHT:
                self.displacement_x = CONTROL_VELOCITY
                self.keys['right'] = True
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                self.displacement_y = 0
                self.keys['up'] = False
            if event.key == pygame.K_DOWN:
                self.displacement_y = 0
                self.keys['down'] = False
            if event.key == pygame.K_LEFT:
                self.displacement_x = 0
                self.keys['left'] = False
            if event.key == pygame.K_RIGHT:
                self.displacement_x = 0
                self.keys['right'] = False
  
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
        self.interacting = False
        self.update()
  
    def display(self):
        self.draw = pygame.draw.circle(SCREEN, self.color, (self.pos_x, self.pos_y), self.radius)
  
    def interaction(self, drone_x, drone_y):
        self.interacting = np.sqrt((drone_x - self.pos_x)**2 + (drone_y - self.pos_y)**2) <= self.radius
        if self.interacting:
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
            self.magnitudes['left'] = str(self.diff_x/HEIGHT * (MAX_RTM - 30) + 30)
            self.magnitudes['right'] = str(0)
        else:
            self.magnitudes['right'] = str(-self.diff_x/HEIGHT * (MAX_RTM - 30) + 30)
            self.magnitudes['left'] = str(0)

        if self.diff_y > 0:
            self.magnitudes['up'] = str(self.diff_y/HEIGHT * (MAX_RTM - 30) + 30)
            self.magnitudes['down'] = str(0)
        else:
            self.magnitudes['down'] = str(-self.diff_y/HEIGHT * (MAX_RTM - 30) + 30)
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

    def reset(self):
        self.magnitudes = {'up':'0', 'right':'0', 'down':'0', 'left':'0'}

def write_header(writer):
    if HAPTICS_ENABLED:
        header = ['frame', 'time', 'drone_x', 'drone_y', 'waypoint_x', 'waypoint_y', 'waypoint_radius', \
                  'interacting', 'rtm_up', 'rtm_down', 'rtm_right', 'rtm_left', 'key_up', 'key_down', \
                  'key_right', 'key_left', 'id', 'visuals', 'haptics']
    else:
        header = ['frame', 'time', 'drone_x', 'drone_y', 'waypoint_x', 'waypoint_y', 'waypoint_radius', \
                  'interacting', 'key_up', 'key_down', 'key_right', 'key_left', 'id', 'visuals', 'haptics']
    
    writer.writerow(header)

def write_data(writer, start_time, frame, id, drone, waypoint, teensy=None):
    if HAPTICS_ENABLED:
        data = [frame, time.time() - start_time, drone.pos_x, drone.pos_y, waypoint.pos_x, waypoint.pos_y, \
                waypoint.radius, waypoint.interacting, teensy.magnitudes['up'], teensy.magnitudes['down'], \
                teensy.magnitudes['right'], teensy.magnitudes['left'], drone.keys['up'], drone.keys['down'], \
                drone.keys['right'], drone.keys['left'], id, VISUALS_ENABLED, HAPTICS_ENABLED]
    else:
        data = [frame, time.time() - start_time, drone.pos_x, drone.pos_y, waypoint.pos_x, waypoint.pos_y, \
                waypoint.radius, waypoint.interacting, drone.keys['up'], drone.keys['down'], \
                drone.keys['right'], drone.keys['left'], id, VISUALS_ENABLED, HAPTICS_ENABLED]
    
    writer.writerow(data)

def main():
    
    if GRAB_ID:
        user_id = input('Please enter user ID: ')
    else:
        user_id = 'NA'

    running = True
  
    # Define objects
    drone = Drone(pos_x=WIDTH//2, pos_y=HEIGHT//2, disturbance_x=0, disturbance_y=0)
    waypoint = Waypoint()
    if HAPTICS_ENABLED:
        teensy = Teensy()
    if DATA_COLLECTION_ENABLED:
        filename = 'Simulation/data/' + time.strftime("%Y%m%d_%H%M%S") + '.csv'
        f = open(filename, 'w')
        writer = csv.writer(f)
        write_header(writer)
    
    # Game Variables
    start_time = time.time()
    frame_count = 0
    paused = START_PAUSED

    while running:

        # Handle User Input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN: 
                if event.key == pygame.K_p:
                    paused = not paused
            if not paused:
                drone.handle(event)

        if not paused:

            # Update Drone Position
            drone.update()

            # Handle Interaction
            waypoint.interaction(drone.pos_x, drone.pos_y)

            # Communicate with Serial
            if HAPTICS_ENABLED:
                if frame_count % 1 == 0:
                    teensy.evaluate(drone, waypoint)
                    teensy.send_serial()

            # Display objects on the SCREEN
            SCREEN.fill(BLACK)
            if VISUALS_ENABLED:
                waypoint.display()
                drone.display()

            if DATA_COLLECTION_ENABLED:
                if HAPTICS_ENABLED:
                    write_data(writer, start_time, frame_count, user_id, drone, waypoint, teensy)
                else:
                    write_data(writer, start_time, frame_count, user_id, drone, waypoint)

            frame_count += 1
  
        if frame_count >= FRAME_LIMIT:
            running = False

        pygame.display.update()
        CLOCK.tick(FPS)  # Adjusting the frame rate

    if DATA_COLLECTION_ENABLED:
        f.close()
    
    if HAPTICS_ENABLED:
        teensy.reset()
        teensy.send_serial()

if __name__ == "__main__":
	main()
	pygame.quit()
        