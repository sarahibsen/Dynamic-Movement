# ----------------------------------------------------------------------------------------------------------------------------------
# author : Sarah Ibsen
# class : CS 330 
# date : 09/22/23

# This program will be implementing and testing the dynamic movement update and three dynamic
# movement behaviors. The three behaviors are seek, flee, and arrive. Second objective is to 
# learn how to plot movement trajectories in matplotlib.

# Requirements: 
#     - Implement dynamic version of the Newton-Euler-1 movement update algorithm (ch 3)
#     - Implement Continue movement behvaior, uses the inital values (continue character's inital velocity and rotation will be 0)
#     - Program should output each character's trajectory as a text file (.txt). Output file should have one 
#     record per character per timestamp, including a record for the initial conditions (time = 0).
# ----------------------------------------------------------------------------------------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
import math



#---------------------------------------------------------------   
Time = 0  # Current simulated time
stop_velocity = 0.02  # Stop moving at velocities below this; avoids jitter
delta_time = 0.5  # Time step for simulation
stop_time = 50.0  # Time to stop simulation
characters = 4
physics = False  # TRUE=HS physics, FALSE=NE1 integration
TIME_STEP = 0.5
charList= {2601, 2602, 2603, 2604}
# Constants for steering output behavior codes
CONTINUE = 1
STOP = 2
SEEK = 6
FLEE = 7
ARRIVE = 8
SIMULATION_TIME = 50.0
AVOID_COLLISIONS = 13
NUM_TIMESTEPS = int(SIMULATION_TIME / TIME_STEP)
#---------------------------------------------------------------
class steering(object):
    def __init__(self):
        self.linear = np.array([0.0, 0.0])
        self.angular = 0


#define a function to calculate orientation based on velocity
def get_orientation(current, velocity):
    if np.linalg.norm(velocity) > 0:
        return np.arctan2(-velocity[0], velocity[1])
    else:
        return current

#---------------------------------------------------------------
# Calculate the closest approach between two moving objects.
# We declare this function in the character class
# def closest_approach(position1, velocity1, position2, velocity2):
#     relative_position = position2 - position1
#     relative_velocity = velocity2 - velocity1

#     time_of_closest_approach = -np.dot(relative_position, relative_velocity) / np.dot(relative_velocity, relative_velocity)

#     relative_position_at_closest_approach = relative_position + time_of_closest_approach * relative_velocity

#     return time_of_closest_approach, np.linalg.norm(relative_velocity), relative_position_at_closest_approach




#---------------------------------------------------------------
#define a class to represent character
class Character(object):
    def __init__(self, ID=0, steer=2, position=np.array([0.0, 0.0]), velocity=np.array([0.0, 0.0]), linear=np.array([0.0, 0.0]), orientation=0.0, rotation=0.0, angular=0.0, maxVelocity=0.0, maxLinear=0.0, maxRotation=0.0, maxAngular=0.0, target=None, arriveRadius=0.0, arriveSlow=0.0, arriveTime=0.0, max_speed=0.0, col_radius=0.0, col_lookahead=0.0, collided=False):
        self.ID = ID
        self.steer = steer
        self.position = position
        self.velocity = velocity
        self.linear = linear
        self.orientation = orientation
        self.rotation = rotation
        self.angular = angular
        self.maxVelocity = maxVelocity
        self.maxLinear = maxLinear
        self.maxRotation = maxRotation
        self.maxAngular = maxAngular
        self.target = target
        self.arriveRadius = arriveRadius
        self.arriveSlow = arriveSlow
        self.arriveTime = arriveTime
        self.max_speed = max_speed
        self.col_radius = col_radius
        self.col_lookahead = col_lookahead
        self.collided = collided
        self.linear_acceleration = np.array([0.0, 0.0])  

     #define a function to update character movement
    def update(self, steering_linear, steering_angular, delta_time, physics):
             
            physics = False #newton-euler-1

            if physics: #high school physics
                halt_t_sq = 0.5 * delta_time * delta_time
                self.position += (self.velocity * delta_time) + (steering_linear * halt_t_sq)
                self.orientation += (self.rotation * delta_time) + (steering_angular * halt_t_sq)
            else: #newton-euler-1
                self.position = self.position + (self.velocity * delta_time)
                self.orientation = self.orientation + (self.rotation * delta_time)
            self.orientation = self.orientation % 360
            self.velocity = self.velocity + (steering_linear * delta_time)
            self.rotation = self.rotation + (steering_angular * delta_time)
            self.linear = steering_linear
            self.angular = steering_angular
            if self.length(self.velocity) > self.maxVelocity:
                self.velocity = self.maxVelocity * normalize(self.velocity)
                #don't need to clip the acceleration because we do it in the bottom 
            # if self.length(self.linear) > self.maxLinear:
            #     self.linear = self.linear * normalize(self.linear)
            if abs(self.rotation) > self.maxRotation:
                self.rotation = self.maxRotation * np.sign(self.rotation)
            if abs(self.angular) > self.maxAngular:
                self.angular = self.maxAngular * np.sign(self.angular)
            return self
#---------------------------------------------------------------
    def update_position(self):
            self.position[0] += self.velocity[0] * TIME_STEP
            self.position[1] += self.velocity[1] * TIME_STEP

    def update_velocity(self):
            self.velocity[0] += self.linear_acceleration[0] * TIME_STEP
            self.velocity[1] += self.linear_acceleration[1] * TIME_STEP
        
    #purse target : move towards a target position at maximum speed, but also predict the target's future position
    def pursueTarget(self, target):
            direction = target.position - self.position
            distance = normalize.length(direction)
            speed = normalize.length(self.velocity)
            if speed <= (distance / self.maxPrediction):
                prediction = self.maxPrediction
            else:
                prediction = distance / speed
            seek = target
            seek.position = seek.position + (target.velocity * prediction)
            self.seek(self, seek)
        #Arrive : move towards a target position at a speed proportional to the distance from the target
        #dont change arrive that shit is good
    def arrive(self, target):
            templinear = np.array([0,0])
            tempangular = 0
            direction = target.position - self.position
            distance = self.length(direction)
            if distance < self.arriveRadius:
                arriveSpeed = 0
            elif distance > self.arriveSlow:
                arriveSpeed = self.maxVelocity
            else:
                arriveSpeed = self.maxVelocity * distance / self.arriveSlow
            arriveVelocity = normalize(direction)*arriveSpeed
            templinear = arriveVelocity - self.velocity
            templinear = templinear / self.arriveTime
            if self.length(templinear) > self.maxLinear:
                templinear = normalize(templinear)
                templinear = templinear * self.maxLinear
                #print(templinear)
            self.linear = templinear
            self.angular = tempangular
            return self.linear, self.angular
    #Seek: move towards a target position at max speed
    def seek(self, target):
            templinear = np.array([0,0])
            templinear = target.position - self.position
            templinear = normalize(templinear)
            templinear = templinear * self.maxLinear
            self.linear = templinear
            self.angular = 0
            return self.linear, self.angular
        
        #Flee : move away from a target position at maximum speed
    def flee(self, target):
        #print(self.position, target.position)
        templinear = np.array([0,0])
        templinear = self.position - target.position
        templinear = normalize(templinear)
        templinear = templinear * self.maxLinear
        self.linear = templinear
        self.angular = 0
        return self.linear, self.angular
    #continue : continue moving without changing velocity or orientation. the initial velocity and rotation will be 0
    def continue_movement(character):
            steering_linear = np.array([0.0, 0.0])
            steering_angular = 0.0
            return steering_linear, steering_angular
        #stop movement : stop moving and rotating
    def stop_movement(self):
                    templinear = self.linear
                    tempangular = 0
                    if self.length(templinear) > self.maxLinear:
                        templinear = normalize(templinear)
                        templinear = templinear * self.maxLinear
                    tempangular = self.angular
                    self.linear = templinear
                    self.angular = tempangular
                    return self.linear, self.angular
    def alignToTarget(self, target):
                templinear = np.array([0,0])
                tempangular = 0
                rotation = target.orientation - self.orientation
                rotation = np.radians(rotation)
                if abs(rotation) < self.alignRadius:
                    tempangular = -tempangular
                if abs(rotation) > self.alignSlow:
                    alignRotation = self.maxRotation
                else:
                    alignRotation = self.maxRotation * abs(rotation) / self.alignSlow
                alignRotation = alignRotation * np.sign(rotation)
                tempangular = (alignRotation - self.rotation) / self.alignTime
                if abs(tempangular) > self.maxAngular:
                    tempangular = self.maxAngular * np.sign(self.angular)
                self.angular = tempangular
                self.linear = templinear
                return self.linear, self.angular
            
    def faceTarget(self,target):
                templinear = np.array([0,0])
                tempangular = 0
                direction = target.position - self.position
                if self.length(direction) == 0:
                    self.linear = templinear
                    self.angular = tempangular
                    return self.linear, self.angular
                target.orientation = np.degrees(np.arctan2(direction[1], direction[0]))
                self.alignToTarget(self, target)
                
    def faceMovement(self):
                templinear = np.array([0,0])
                tempangular = 0
                if self.length(self.velocity) == 0:
                    self.linear = templinear
                    self.angular = tempangular
                    return self.linear, self.angular
                target = self
                target.orientation = np.degrees(np.arctan2(self.velocity[1], self.velocity[0]))
                self.alignToTarget(self, target)
    def closestApproach(self, target):
                # Relative velocity and position
                relative_velocity = self.velocity - target.velocity
                relative_position = self.position - target.position
                
                # Time at which the distance is minimized
                t_min = -np.dot(relative_position, relative_velocity) / np.dot(relative_velocity, relative_velocity)
                
                # Positions at t_min
                self_position_at_t_min = self.position + self.velocity * t_min
                target_position_at_t_min = target.position + target.velocity * t_min
                
                # Minimum distance
                min_distance = np.linalg.norm(self_position_at_t_min - target_position_at_t_min)
                
                # Relative positions at t_min
                relative_position_at_t_min = self_position_at_t_min - target_position_at_t_min
                
                return t_min, min_distance, relative_position_at_t_min
    def avoid(self, lookahead):
                colFound = False
                colTime = np.inf
                colPosition = 0

                for character in self.characters:
                    target = character
                    closeApproachTime, closeApproachDistance, relativeDistance = self.closestApproach(self, target)
                    if closeApproachTime > 0 and closeApproachTime < colTime and closeApproachTime < lookahead and closeApproachDistance < (self.avoidRadius + target.avoidRadius):
                        colFound = True
                        colTime = closeApproachTime
                        colPosition = relativeDistance
                templinear = np.array([0,0])
                tempangular = 0
                if colFound:
                    colPosition = self.length(colPosition)
                    templinear = colPosition * self.maxLinear
                self.linear = templinear
                self.angular = tempangular
                return self.linear, self.angular
    #helper function to calculate the length of a vector 
    def length(self, vector):
            length = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])
            return length
        #normalize vector to unit length
def normalize(vector):
     # Calculate the magnitude of the vector
    magnitude = np.linalg.norm(vector)
    
    # Avoid division by zero by returning a zero vector if the magnitude is zero
    if magnitude == 0:
        return vector
    
    # Normalize the vector by dividing each component by the magnitude
    normalized_vector = vector / magnitude
    
    return normalized_vector
     
#---------------------------------------------------------------
#calculate the trajectory of each character, timestep by timestep
characters = []

character01 = Character(ID = 2601, steer = 1, position = np.array([0.0, 0.0]), velocity = np.array([0.0, 0.0]), linear = np.array([0.0, 0.0]), orientation = 0.0, rotation = 0.0, angular = 0.0, maxVelocity = 0.0, maxLinear = 0.0, maxRotation = 0.0, maxAngular = 0.0, target = None, arriveRadius = 0.0, arriveSlow = 0.0, arriveTime = 0.0, max_speed = 0.0, col_radius = 0.0, col_lookahead = 0.0, collided = False)
character02 = Character(ID = 2602, steer = 7, position = np.array([-30.0, -50.0]), velocity = np.array([2.0, 7.0]), linear = np.array([0.0, 0.0]), orientation = (math.pi / 4.0), rotation = 0.0, angular = 0.0, maxVelocity = 8.0, maxLinear = 1.5, maxRotation = 0.0, maxAngular = 0.0, target = character01, arriveRadius = 0.0, arriveSlow = 0.0, arriveTime = 0.0, max_speed = 0.0, col_radius = 0.0, col_lookahead = 0.0, collided = False)
character03 = Character(ID = 2603, steer = 6, position = np.array([-50.0, 40.0]), velocity = np.array([0.0, 8.0]), linear = np.array([0.0, 0.0]), orientation = (math.pi*(3.0) / 2.0), rotation = 0.0, angular = 0.0, maxVelocity = 8.0, maxLinear = 2.0, maxRotation = 0.0, maxAngular = 0.0, target = character01, arriveRadius = 0.0, arriveSlow = 0.0, arriveTime = 0.0, max_speed = 0.0, col_radius = 0.0, col_lookahead = 0.0, collided = False)
character04 = Character(ID = 2604, steer = 8, position = np.array([50.0, 75.0]), velocity = np.array([-9.0, 4.0]), linear = np.array([0.0, 0.0]), orientation = (math.pi), rotation = 0.0, angular = 0.0, maxVelocity = 10.0, maxLinear = 2.0, maxRotation = 0.0, maxAngular = 0.0, target = character01, arriveRadius = 4.0, arriveSlow = 32.0, arriveTime = 1.0, max_speed = 0.0, col_radius = 0.0, col_lookahead = 0.0, collided = False)
characters.append(character01)
characters.append(character02)
characters.append(character03)
characters.append(character04)

# Modify the trajectory file to print data without header and as natural types
with open("movement_data.txt", 'w') as file:
    for character in characters:
        print(
            Time,
            character.ID,
            character.position[0],
            character.position[1],
            character.velocity[0],
            character.velocity[1],
            character.linear[0],
            character.linear[1],
            character.orientation,
            character.steer,
            character.collided,
            sep=",",
            end="\n",
            file=file
        )

#main simulation loop
time = 0
while time < stop_time:
    time = time + delta_time
    for character in characters:
        # Calculate steering for the current character based on its behavior
        if character.steer == CONTINUE:
            steering_linear, steering_angular = character.continue_movement()
        elif character.steer == STOP:
            steering_linear, steering_angular = character.stop_movement()
        elif character.steer == "ALIGN":
            target = character.target
            steering_linear, steering_angular = character.alignToTarget(target)
        elif character.steer == "FACE_TARGET":
            target = character.target
            steering_linear, steering_angular = character.faceTarget(target)
        elif character.steer == "FACE_MOVEMENT":
            steering_linear, steering_angular = character.faceMovement()
        elif character.steer == 6:
            target = character.target
            steering_linear, steering_angular = character.seek(target)
        elif character.steer == 7:
            target = character.target
            steering_linear, steering_angular = character.flee(target)
        elif character.steer == ARRIVE:
            target = character.target
            steering_linear, steering_angular = character.arrive(target)
        elif character.steer == "PURSUE":
            target = character.target
            steering_linear, steering_angular = character.pursueTarget(target)
        elif character.steer == "AVOID":
            steering_linear, steering_angular = character.avoid(character.colLookahead)
        
        # Update the character's properties based on the calculated steering
        character.update(steering_linear, steering_angular, delta_time, physics)


       # 
        with open("movement_data.txt", 'a') as file:  
            print(
                time,
                character.ID,
                character.position[0],
                character.position[1],
                character.velocity[0],
                character.velocity[1],
                character.linear[0],
                character.linear[1],
                character.orientation,
                character.steer,
                character.collided,
                sep=",",
                end="\n",
                file=file
            )
        #update(character, character.steer, delta_time, physics)
 


print("Simulation data written to 'movement_data.txt'.")