#!/usr/bin/python -tt

# BZRC Imports
from bzrc import BZRC, Command

# OpenGL Imports
import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
#from numpy import zeros
import numpy

# Misc Imports
import sys, math, time, random


def normalize_angle(angle):
    '''Make any angle be between +/- pi.'''
    angle -= 2 * math.pi * int (angle / (2 * math.pi))
    if angle <= -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle
    
def dist(pt1, pt2):
    '''Calculate distance between two points'''
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    
def midpoint(pt1, pt2):
    return ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2)
    
def add(vector1, vector2):
    return (vector1[0] + vector2[0], vector1[1] + vector2[1])
    
# grid is a numpy grid, center is a point in the grid to center at, size is the size of the box that should be averaged
def subgrid(grid, center, size):
    width, height = grid.shape
    left = int(max(0, center[0] - size/2))
    right = int(min(width, center[0] + size/2))
    bottom = int(max(0, center[1] - size/2))
    top = int(min(height, center[1] + size/2))
    return grid[bottom:top, left:right]
    
def average(l):
    return reduce(lambda x, y: x + y, l) / len(l)

def average_grid(grid, center, size):
    return numpy.mean(subgrid(grid, center, size))
    
def min_grid(grid, center, size):
    return numpy.min(subgrid(grid, center, size))
    
def max_grid(grid, center, size):
    return numpy.max(subgrid(grid, center, size))

def deg2rad(n):
    return n * (math.pi / 180.0)
    
def rad2deg(n):
    return n * (180.0 / math.pi)
    

class Agent(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        
        # Initialize World Map / Belief Grid
        init_window(int(self.constants["worldsize"]), int(self.constants["worldsize"]))
        self.bel_grid = numpy.array(list(list(.75 for j in range(int(self.constants["worldsize"]))) for i in range(int(self.constants["worldsize"]))))
        self.conf_grid = numpy.array(list(list(0.0 for j in range(int(self.constants["worldsize"]))) for i in range(int(self.constants["worldsize"]))))
        update_grid_display(self.conf_grid)
        
        self.commands = []
        self.tanks = {tank.index:Tank(bzrc, self, tank) for tank in self.bzrc.get_mytanks()}
        
        ''' Available Constants ''' '''
        CONSTANT        EX. OUTPUT
        team            blue
        worldsize       800
        hoverbot        0
        puppyzone       30
        tankangvel      0.785398163397
        tanklength      6
        tankradius      4.32
        tankspeed       25
        tankalive       alive
        tankdead        dead
        linearaccel     0.5
        angularaccel    0.5
        tankwidth       2.8
        shotradius      0.5
        shotrange       350
        shotspeed       100
        flagradius      2.5
        explodetime     5
        truepositive    1
        truenegative    1
        '''
    
    def update_belief(self, row, col, grid_val):
        '''Update the belief grid based on Bayes Rule'''
        if grid_val == 1:  # if we observe a hit
            Bel_Occ = float(self.constants["truepositive"]) * self.bel_grid[row,col]
            Bel_Unocc = (1-float(self.constants["truenegative"])) * (1-self.bel_grid[row,col])
            # Normalize
            self.bel_grid[row,col] = Bel_Occ / (Bel_Occ + Bel_Unocc);
            
        else:  # If do not observe a hit
            Bel_Occ = (1-float(self.constants["truepositive"])) * self.bel_grid[row,col]
            Bel_Unocc = float(self.constants["truenegative"]) * (1-self.bel_grid[row,col])
            # Normalize
            self.bel_grid[row,col] = Bel_Occ / (Bel_Occ + Bel_Unocc)
    
    
    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        
        # ALGORITHM
        # 
        # for all_tanks
        #  if should_sample()
        #   call occgrid
        #   update bel/conf grid
        #  if picknewpoint()
        #   picknewpoint()
        #  movetopoint()
        # 
        
        # Reset my set of commands (we don't want to run old commands)
        self.commands = []
        
        # Decide what to do with each of my tanks
        width, height = self.conf_grid.shape
        average_confidence = numpy.average(self.conf_grid)
        for bot in self.bzrc.get_mytanks():
            # Get the tank and update it with what we received from the server
            tank = self.tanks[bot.index]
            tank.update(bot)
            
            # Check to see if the tank should take a sample now
            if tank.should_sample(self.conf_grid):
                # Call occgrid
                pos, grid = self.bzrc.get_occgrid(bot.index)
                
                # Iterate over each cell in the sampled grid
                for col in range(pos[0] + 400, pos[0] + 400 + grid.shape[0]):
                    for row in range(pos[1] + 400, pos[1] + 400 + grid.shape[1]):
                        #print col, row
                        # Update belief grid
                        self.update_belief(row, col, grid[col-pos[0]-400,row-pos[1]-400])
                        # Update confidence grid
                        self.conf_grid[row, col] += .1
                
                # This code is replaced with the loop above
                # Update confidence grid
                # With the rounding here, this might have some off by one errors...when adding the updates for the belief grid, this should be changed as well, so that only the cells changed are updated.
                #for col in range(int(max(0, tank.x + 350)), int(min(width, tank.x + 450))): # TODO: AGAIN: THESE ASSUME A CENTERED, 800x800 WORLD
                #    for row in range(int(max(0, tank.y + 350)), int(min(height, tank.y + 450))):
                #        #self.conf_grid[row, col] += .1
                #        pass
                #self.conf_grid = numpy.add(self.conf_grid, 1)
                
            tank.check_pick_new_point(average_confidence, time_diff)
            
            self.commands.append(tank.get_desired_movement_command(time_diff, int(self.constants["tankspeed"])))

        # Send the movement commands to the server
        results = self.bzrc.do_commands(self.commands)
        
        update_grid_display(self.bel_grid)
    
class Tank(object):
    
    def __init__(self, bzrc, agent, tank):
        self.bzrc = bzrc
        self.agent = agent
        self.previous_error_angle = 0
        self.previous_error_speed = 0
        self.x = None
        self.y = None
        self.update(tank)
        self.prev_x = self.x
        self.prev_y = self.y
        self.target = (random.randint(-400, 400), random.randint(-400, 400))
        #self.pick_point(0)
    
    def update(self, tank):
        self.prev_x = self.x
        self.prev_y = self.y
        self.index = tank.index;
        self.callsign = tank.callsign;
        self.status = tank.status;
        self.shots_avail = tank.shots_avail;
        self.time_to_reload = tank.time_to_reload;
        self.flag = tank.flag;
        self.x = tank.x;
        self.y = tank.y;
        self.angle = tank.angle;
        self.vx = tank.vx;
        self.vy = tank.vy;
        self.angvel = tank.angvel;
        
    def check_pick_new_point(self, average_confidence, time_diff):
        delta_x = self.target[0] - self.x
        delta_y = self.target[1] - self.y
        #error_angle = normalize_angle(math.atan2(delta_y, delta_x) - normalize_angle(self.angle));
        #print "speed:",dist((self.vx, self.vy), (0, 0))/time_diff
        #if dist((self.x, self.y), self.target) < 20 or (dist((self.vx, self.vy), (0, 0))/time_diff < 1 and math.fabs(error_angle) < math.pi/6):
        
        magnitude = math.sqrt(delta_x**2 + delta_y**2)
        direction = (delta_x / magnitude, delta_y / magnitude)
        
        if dist((self.x, self.y), self.target) < 20 or (average_grid(self.agent.bel_grid, (self.x + 5 * direction[0] + 400, self.y + 5 * direction[1] + 400), 10) > .8) or (self.x == self.prev_x and self.y == self.prev_y):
            # We need a new point
            self.pick_point(average_confidence)
        else:
            #print "no new target"
            pass
        
    def pick_point(self, average_confidence):
        #newx = random.randint(-400, 400)
        #newy = random.randint(-400, 400)
        #attempts = 0
        #while average_grid(self.agent.conf_grid, (newx + 400, newy + 400), 100) > average_confidence and attempts < 5:
        #    attempts += 1
        #    newx = random.randint(-400, 400)
        #    newy = random.randint(-400, 400)
        #self.target = (newx, newy)
        self.target = self.pick_recursive(self.agent.conf_grid, 0, 800, 0, 800, 4)
        #print "New target:", self.target
        
    def pick_recursive(self, grid, left, right, bottom, top, depth):
        if depth == 0:
            return (random.randint(left, right)-400, random.randint(bottom, top)-400)

        width = right - left
        height = top - bottom
        left_split = numpy.average(grid[bottom:top, left + width/2:right])
        right_split = numpy.average(grid[bottom:top, left:right - width/2])
        bottom_split = numpy.average(grid[bottom + height/2:top, left:right])
        top_split = numpy.average(grid[bottom:top - height/2, left:right])
        min_avg = min(left_split, right_split, bottom_split, top_split)
        if left_split == min_avg:
            return self.pick_recursive(grid, left + width/2, right, bottom, top, depth - 1)
        elif right_split == min_avg:
            return self.pick_recursive(grid, left, right - width/2, bottom, top, depth - 1)
        elif bottom_split == min_avg:
            return self.pick_recursive(grid, left, right, bottom + height/2, top, depth - 1)
        else:
            return self.pick_recursive(grid, left, right, bottom, top - height/2, depth - 1)
        
    def should_sample(self, conf_grid):
        # TODO: THE 400s HERE ARE TO MOVE COORDINATES INTO ALL POSITIVE SPACE. THE REGULAR WORLD IS CENTERED AROUND THE POINT (0, 0)
        # this 10 is our hard coded decision of when we feel like we won't get any more benefit from sampling this region
        confidence_threshold = .1
        return average_grid(conf_grid, (self.x + 400, self.y + 400), 100) < confidence_threshold
    
    def get_desired_movement_command(self, time_diff, maxspeed):
        # PD Controller stuff to make movement smoother
        delta_x = self.target[0] - self.x
        delta_y = self.target[1] - self.y
        #print "Delta:", (delta_x, delta_y)
        
        target_angle = math.atan2(delta_y, delta_x)
        current_angle = normalize_angle(self.angle)
        error_angle = normalize_angle(target_angle - current_angle);
        #print "Error:", int(rad2deg(error_angle)), "Target:", int(rad2deg(target_angle)), "Current:", int(rad2deg(current_angle))
        # clamp the speed to -1 to 1 (technically, 0 to 1)
        # Base the speed on the current angle as well
        target_speed = math.cos(error_angle) * maxspeed
        current_speed = math.sqrt(math.pow(self.vy, 2) + math.pow(self.vx, 2))
        error_speed = target_speed - current_speed;
        #print "Error:", int(error_speed), "Target:", int(target_speed), "Current:", int(current_speed)
        
        proportional_gain_angle = 2.25
        proportional_gain_speed = 1.0
        derivative_gain_angle = 0.5
        derivative_gain_speed = 0.1
        
        send_angvel = proportional_gain_angle * error_angle + derivative_gain_angle * ((error_angle - self.previous_error_angle) / time_diff)
        send_speed = proportional_gain_speed * error_speed + derivative_gain_speed * ((error_speed - self.previous_error_speed) / time_diff)
        
        self.previous_error_angle = error_angle
        self.previous_error_speed = error_speed
        
        magnitude = math.sqrt(delta_x**2 + delta_y**2)
        direction = (delta_x / magnitude, delta_y / magnitude)
        
        #dist((self.vx, self.vy), (0, 0))/time_diff < 1 and math.fabs(error_angle) < math.pi/6: # Did we not move very far, and were we facing the right way?
        if average_grid(self.agent.bel_grid, (self.x + 5 * direction[0] + 400, self.y + 5 * direction[1] + 400), 10) > .8 or (self.x == self.prev_x and self.y == self.prev_y): # Are we reasonably sure we're running into an obstacle right now?
            # If we are hitting an obstacle, send the max angular velocity
            send_angvel = 1
        #    print "true"
        #else:
        #    print "false"
            
        return Command(self.index, send_speed, send_angvel, 0)

# OpenGL functions
window = None
grid = None

def draw_grid():
    # This assumes you are using a numpy array for your grid
    width, height = grid.shape
    glRasterPos2f(-1, -1)
    glDrawPixels(width, height, GL_LUMINANCE, GL_FLOAT, grid)
    glFlush()
    glutSwapBuffers()

def update_grid_display(new_grid):
    global grid
    grid = new_grid
    draw_grid()

def init_window(width, height):
    global window
    global grid
    grid = numpy.zeros((width, height))
    glutInit(())
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutInitWindowPosition(0, 0)
    #window = glutCreateWindow("Grid filter lab: Confidence")
    window = glutCreateWindow("Grid filter lab: Belief")
    glutDisplayFunc(draw_grid)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    #glutMainLoop()
        
def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)
        
    #print "Running Tyler & Morgan's Super Smart Flag Capturer"

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))
    
    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()

if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4