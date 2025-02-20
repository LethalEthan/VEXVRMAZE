#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
# 
# ------------------------------------------

# Add project code in "main"
def advanced():
    pen.move(DOWN)
    pen.set_pen_color(BLUE)
    pen.set_pen_width(THIN)
    drivetrain.set_drive_velocity(100,PERCENT)
    drivetrain.set_turn_velocity(100,PERCENT)
    initialPosition = [location.position(X,MM),location.position(Y,MM),drivetrain.heading(DEGREES)]
    # Create 8x8 matrix
    array_8 = [[0 for _ in range(8)] for _ in range(8)]
    first_run = True
    north = 0x01
    east = 0x02
    south = 0x04
    west = 0x08
    visited = 0x10
    start = 0x20
    end = 0x30
    # dead end bitmasks
    new = north|east|west
    nes = north|east|south
    esw = east|south|west
    nsw = north|south|west
    passes = 0 # passes over start pos
    while passes < 2:
        wait(100, MSEC)
        walls = 0
        newpos = positionToMatrixIndex(location.position(X,MM),location.position(Y,MM)) # translate mm to index
        inital_heading = drivetrain.heading(DEGREES)
        if first_run: # mark on first run start position
            walls = walls | start
            walls = walls | south
            first_run = False
            brain.print("marked start")
        if not first_run:
            if array_8[newpos[0]][newpos[1]] & start: # if we reach start again
                passes+=1
                walls = walls | south
                brain.print("increment")
                if passes >= 2:
                    brain.print("breaking")
                    break
                visit_count = 0
                for x in range(len(array_8)):
                    for y in range(len(array_8[0])):
                        if array_8[x][y] & visited:
                            visit_count += 1
                            brain.new_line()
                            brain.print("visit_count: ")
                            brain.print(visit_count)
                if visit_count >= 64:
                    brain.print("visited all maze")
                    break

        # check directions
        drivetrain.turn_to_heading(0,DEGREES) # north
        if front_distance.get_distance(MM) < 100:
            walls = walls | north
            brain.print("mark north ")
        drivetrain.turn_to_heading(90,DEGREES) #east
        if front_distance.get_distance(MM) < 100:
            walls = walls | east
            brain.print("mark east ")
        drivetrain.turn_to_heading(180,DEGREES)#south
        if front_distance.get_distance(MM) < 100:
            walls = walls | south
            brain.print("mark south ")
        drivetrain.turn_to_heading(270,DEGREES) #west
        if front_distance.get_distance(MM) < 100:
            walls = walls | west
            brain.print("mark west ")
        # mark north wall when we reach the end, prevent falling off
        if down_eye.detect(RED) and newpos[1] == 7: # Y must be 7 which is where the end always is
            brain.print("reached end")
            walls = walls | north # mark above finish as wall
        
        #
        # dead end logic
        #
        if walls == new or walls == new | end or walls == new | start: # if walls are north, east and west set heading to south as we hit dead end
            brain.print("gonna face south now")
            drivetrain.turn_to_heading(180,DEGREES) # face south
        elif walls == nes or walls == nes | end or walls == nes | start:
            brain.print("gonna face west now")
            drivetrain.turn_to_heading(270,DEGREES) # face west
        elif walls == esw or walls == esw | end or walls == esw | start:
            brain.print("gonna face north now")
            drivetrain.turn_to_heading(0,DEGREES) # face north
        elif walls == nsw or walls == nsw | end or walls == nsw | start:
            brain.print("gonna face east now")
            drivetrain.turn_to_heading(90,DEGREES) # face east
        else:
            drivetrain.turn_to_heading(inital_heading,DEGREES) # use the initial heading if we're not at a dead end
        # mark position as visited
        walls = walls | visited
        #
        # mark the end position, we do it here because it interferes with the dead end facing logic
        #
        if down_eye.detect(RED) and newpos[1] == 7: # Y must be 7 which is where the end always is
            brain.print("marked end")
            walls = walls | end
        if not array_8[newpos[0]][newpos[1]] & visited: # don't store if we already visited
            brain.new_line()
            brain.print("stored in map")
            array_8[newpos[0]][newpos[1]] = walls # store wall bitmask in matrix
        brain.new_line()
        #
        # left hand  wall follow to map whole maze
        #
        drivetrain.turn_for(LEFT,90,DEGREES)      
        if front_distance.get_distance(MM) < 100:
            drivetrain.turn_for(RIGHT,90,DEGREES)
            if front_distance.get_distance(MM) < 100:
                drivetrain.turn_for(RIGHT,90,DEGREES)
        # start position, prevent falling off
        if array_8[newpos[0]][newpos[1]] & start and front_distance.get_distance(MM) > 2000 and newpos[1] == 0:
            drivetrain.turn_for(RIGHT,90,DEGREES)
        # end position, prevent falling off
        #if array_8[newpos[0]][newpos[1]] & end and front_distance.get_distance(MM) > 2000:
        #    drivetrain.turn_for(RIGHT,90,DEGREES)
        drivetrain.drive_for(FORWARD,250,MM)
    print_maze(array_8)

def print_maze(maze):
    # bitmasks
    north = 0x01
    east = 0x02
    south = 0x04
    west = 0x08
    visited = 0x10
    start = 0x20
    end = 0x30
    brain.new_line()
    h, w = len(maze), len(maze[0]) # height and width of maze

    for y in range(h):
        top_row = ""
        mid_row = ""
        for x in range(w):
            if maze[x][y] & north:  # Top wall
                top_row += "+---"
            else:
                top_row += "+   "
            if maze[x][y] & west:
                mid_row += "|   "
            #else:
            #    mid_row += "|   "

            if x == w - 1:  # Right wall
                if maze[x][y] & east:
                    mid_row += "|"

        top_row += "+"
        mid_row += "|"
        brain.print(top_row)
        brain.new_line()
        brain.print(mid_row)
        brain.new_line()
        if y == h - 1:  # Bottom row
            bottom_row = ""
            for x in range(w):
                if maze[x][y] & south:
                    bottom_row += "+---"
                else:
                    bottom_row += "+   "
            bottom_row += "+"
            brain.print(bottom_row)
        
    brain.new_line()
    brain.new_line()
    brain.new_line()
    brain.new_line()
            
    #for y in range(7, -1, -1):
    #    row = ""
    #    for x in range(w):    
    #        if maze[x][y] & west:
    #            row += "|"
    #        if maze[x][y] & north:
    #            row += "-"
            #if maze[x][y] & start:
            #    row += "S"
            #if maze[x][y] & end:
            #    row += "E"
   #         brain.print(row)
    #        brain.new_line()

            

# we translate the mm position to an index of the 8x8 matrix
def positionToMatrixIndex(x,y):
    new_x = 0
    new_y = 0
    if x > -900 and x <= -650:
        new_x = 0
    if x > -650 and x <= -400:
        new_x = 1
    if x > -400 and x <= -150:
        new_x = 2
    if x > -150 and x <= 100:
        new_x = 3
    if x > 100 and x <= 350:
        new_x = 4
    if x > 350 and x <= 600:
        new_x = 5
    if x > 600 and x <= 850:
        new_x = 6
    if x > 850 and x <= 1000:
        new_x = 7
    #y
    if y > -900 and y <= -650:
        new_y = 0
    if y > -650 and y <= -400:
        new_y = 1
    if y > -400 and y <= -150:
        new_y = 2
    if y > -150 and y <= 100:
        new_y = 3
    if y > 100 and y <= 350:
        new_y = 4
    if y > 350 and y <= 600:
        new_y = 5
    if y > 600 and y <= 850:
        new_y = 6
    if y > 850 and y <= 1000:
        new_y = 7
    return new_x,new_y

# Just left hand wall following
def basic():  
    pen.move(DOWN)
    pen.set_pen_color(BLUE)
    pen.set_pen_width(THIN)
    drivetrain.set_drive_velocity(100,PERCENT)
    drivetrain.set_turn_velocity(100,PERCENT)
    while True:
        wait(100, MSEC)
        drivetrain.turn_for(LEFT,90,DEGREES)      
        if front_eye.detect(RED):
            drivetrain.turn_for(RIGHT,90,DEGREES)
            if front_eye.detect(RED):
                drivetrain.turn_for(RIGHT,90,DEGREES)
        drivetrain.drive_for(FORWARD,250,MM)
        if down_eye.detect(RED):
            break

# VR threads — Do not delete
vr_thread(advanced)
