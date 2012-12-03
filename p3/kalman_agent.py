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
    
def angle_from(pt1, pt2):
    # Return angle from pt1 to pt2
    return math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0])
    
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
    
class KFilter(object):
    # Currently assumes 6 variables in the state: x_pos,x_vel,x_acc,y_pos,y_vel,y_acc
    def __init__(self, mu0 = numpy.zeros((6,1), dtype='float64'), Sigma0 = None, pnoise=5):
        # Constants
        self.Sigma_x = numpy.array([[.1,   0,    0,   0,   0,     0],\
                                    [ 0,  .1,    0,   0,   0,     0],\
                                    [ 0,   0,   10,  0,   0,     0],\
                                    [ 0,   0,    0,  .1,   0,     0],\
                                    [ 0,   0,    0,   0,  .1,     0],\
                                    [ 0,   0,    0,   0,   0,   10]], dtype='float64')
        self.Sigma_z = numpy.array([[pnoise**2, 0],\
                                    [0, pnoise**2]], dtype='float64')
        
        # t-dependent
        self.gain_t = numpy.zeros((6,2))
        self.mu_t = mu0
        if Sigma0:
            self.Sigma_t = Sigma0
        else:
            self.Sigma_t = numpy.array([[100,   0,   0,  0,   0,     0],\
                                        [0,     .1,  0,  0,   0,     0],\
                                        [0,     0,  .1,  0,   0,     0],\
                                        [0,     0,  0,   100, 0,     0],\
                                        [0,     0,  0,   0,   .1,    0],\
                                        [0,     0,  0,   0,   0,     .1]], dtype='float64')
    
    def _calc_gain(self, M):
        temp1 = M.dot(self.HT)
        temp2 = self.H.dot(M).dot(self.HT) + self.Sigma_z
        temp3 = temp1.dot(numpy.linalg.inv(temp2))
        return temp3
    
    def _calc_mu_t(self, z_t):
        temp1 = z_t - self.H.dot(self.F).dot(self.mu_t)
        temp2 = self.F.dot(self.mu_t) + self.gain_t.dot(temp1)
        return temp2
    
    def _calc_sigma_t(self, M):
        temp1 = numpy.identity(6) - self.gain_t.dot(self.H)
        temp2 = temp1.dot(M)
        return temp2
    
    def run(self, t, z_t):
        print "Sigma_t pre:"
        print self.Sigma_t
        
        # Calculate F and H and their transposes
        self.F = numpy.array([[1,   t, t**2/2, 0,   0,      0],\
                              [0,   1,      t, 0,   0,      0],\
                              [0, -.0,      1, 0,   0,      0],\
                              [0,   0,      0, 1,   t, t**2/2],\
                              [0,   0,      0, 0,   1,      t],\
                              [0,   0,      0, 0, -.0,      1]], dtype='float64')
        self.FT = numpy.transpose(self.F)
        self.H = numpy.array([[1,0,0,0,0,0],\
                              [0,0,0,1,0,0]], dtype='float64')
        self.HT = numpy.transpose(self.H)
        
        # Calculate F(sigma_t)FT + sigma_x
        M = self.F.dot(self.Sigma_t).dot(self.FT) + self.Sigma_x
        
        # Kalman Gain
        self.gain_t = self._calc_gain(M)
        #print "Gain:"
        #print self.gain_t
        
        # mu_t (Best Guess)
        self.mu_t = self._calc_mu_t(z_t)
        #print "Mu_t:"
        #print self.mu_t
        
        # Sigma_t (Uncertainty)
        self.Sigma_t = self._calc_sigma_t(M)
        print "Sigma_t:"
        print self.Sigma_t
        
    
    def predict(self, t):
        tempF = numpy.array([[1,   t, t**2/2, 0,   0,      0],\
                              [0,   1,      t, 0,   0,      0],\
                              [0, -.1,      1, 0,   0,      0],\
                              [0,   0,      0, 1,   t, t**2/2],\
                              [0,   0,      0, 0,   1,      t],\
                              [0,   0,      0, 0, -.1,      1]], dtype='float64')
        return tempF.dot(self.mu_t)
    
    def getPosition(self):
        return (self.mu_t[0][0],self.mu_t[3][0])
    def getVelocity(self):
        return (self.mu_t[1][0],self.mu_t[4][0])
    def getPosUncertainty(self):
        return (self.Sigma_t[0][0], self.Sigma_t[3][3])
        
    
class Agent(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        
        # Initialize our Enemy (as a k_filter)
        self.k_enemy = KFilter()
        
        
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
        
        self.file_suffix = 0
        self.file_write_time_accumulator = 0
        #self.write_kalman_fields("test1", 10, 10, 0, 0)
        #self.write_kalman_fields("test2", 100, 10, 0, 0)
        #self.write_kalman_fields("test3", 10, 100, 0, 0)
        #self.write_kalman_fields("test4", 100, 100, .5, 0)
        #self.write_kalman_fields("test5", 100, 100, .9999, 0)
    
    def write_kalman_fields(self, file, sigma_x, sigma_y, rho, time_diff):
        self.file_write_time_accumulator += time_diff
        if self.file_write_time_accumulator > 1 and sigma_x > 0 and sigma_y > 0:
            self.file_write_time_accumulator = 0
            self.file_suffix += 1
            with open("{0}-{1}.gpi".format(file, self.file_suffix), 'w+') as out:
                # header
                out.write("set xrange [-400.0: 400.0]\n")
                out.write("set yrange [-400.0: 400.0]\n")
                out.write("set pm3d\n")
                out.write("set view map\n")
                out.write("unset key\n")
                out.write("set size square\n")
                # Print to png when run
                out.write("set term png\n")
                out.write("set output \"{0}-{1}.png\"\n".format(file, self.file_suffix))
                out.write("\n")
                out.write("unset arrow\n")
                #out.write("set arrow from 0, 0 to -150, 0 nohead front lt 3\n")
                #out.write("set arrow from -150, 0 to -150, -50 nohead front lt 3\n")
                #out.write("set arrow from -150, -50 to 0, -50 nohead front lt 3\n")
                #out.write("set arrow from 0, -50 to 0, 0 nohead front lt 3\n")
                #out.write("set arrow from 200, 100 to 200, 330 nohead front lt 3\n")
                #out.write("set arrow from 200, 330 to 300, 330 nohead front lt 3\n")
                #out.write("set arrow from 300, 330 to 300, 100 nohead front lt 3\n")
                #out.write("set arrow from 300, 100 to 200, 100 nohead front lt 3\n")
                out.write("\n")
                out.write("set palette model RGB functions 1-gray, 1-gray, 1-gray\n")
                out.write("set isosamples 100\n")
                out.write("\n")
                out.write("sigma_x = {0}\n".format(sigma_x))
                out.write("sigma_y = {0}\n".format(sigma_y))
                out.write("rho = {0}\n".format(rho))
                out.write("splot 1.0/(2.0 * pi * sigma_x * sigma_y * sqrt(1 - rho**2) ) * exp(-1.0/2.0 * (x**2 / sigma_x**2 + y**2 / sigma_y**2 - 2.0*rho*x*y/(sigma_x*sigma_y) ) ) with pm3d\n")
    
    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        
        # Reset my set of commands (we don't want to run old commands)
        self.commands = []
        
        # ALGORITHM
        # 
        # Read input on enemy tanks
        # Run kalman filter to update enemy tanks
        # for all mytanks
        #     update target
        #     update target_velocity
        #     calculate movement_command
        # send commands
        
        enemy = self.bzrc.get_othertanks()[0]
        z_t = numpy.array([[numpy.float64(enemy.x)],\
                           [numpy.float64(enemy.y)]], dtype='float64')
        if enemy.status == "dead":
            self.k_enemy = None
        else:
            if self.k_enemy == None:
                self.k_enemy = KFilter()
            #print "Time Diff: ", time_diff
            #print "z_t:"
            #print z_t
            self.k_enemy.run(time_diff, z_t)
            
            for bot in self.bzrc.get_mytanks():
                # Get the tank and update it with what we received from the server
                tank = self.tanks[bot.index]
                tank.update(bot)
                tank.target = self.k_enemy.getPosition()
                tank.target_velocity = self.k_enemy.getVelocity()
                
                self.commands.append(tank.get_desired_movement_command(time_diff, 0))

            # Send the movement commands to the server
            results = self.bzrc.do_commands(self.commands)
            
            pos_u = self.k_enemy.getPosUncertainty()
            self.write_kalman_fields("kalman", pos_u[0], pos_u[1], 0, time_diff)
    
class Tank(object):
    
    def __init__(self, bzrc, agent, tank):
        self.bzrc = bzrc
        self.agent = agent
        self.k_enemy = self.agent.k_enemy
        self.previous_error_angle = 0
        self.previous_error_speed = 0
        self.x = None
        self.y = None
        self.update(tank)
        self.prev_x = self.x
        self.prev_y = self.y
        # These variables are now used to point to the currently estimated position of the enemy tank, and its current velocity (x and y components)
        self.target = (0, 0)
        self.target_velocity = (0, 0)
        #print "Initial Target:", self.target
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
        
    def estimate_firing_angle(self):
        # Thanks, http://playtechs.blogspot.com/2007/04/aiming-at-moving-target.html
        px = self.target[0] - self.x
        py = self.target[1] - self.y
        vx = self.target_velocity[0]
        vy = self.target_velocity[1]
        s = 100 #bullet speed
        
        a = s * s - (vx * vx + vy * vy)
        b = px * vx + py * vy
        c = px * px + py * py
        d = b * b + a * c
        t = 0
        if d >= 0:
            t = (b + math.sqrt(d)) / a
            if t < 0:
                print "T was less than zero?"
                t = 0
        
        #prediction = self.k_enemy.predict(t)
        #shoot_at = (prediction[0][0], prediction[3][0])
        shoot_at = (self.target[0] + self.target_velocity[0] * t, self.target[1] + self.target_velocity[1] * t)
        
        angle = angle_from((self.x, self.y), shoot_at)
        
        print "Target:", self.target
        print "Velocity:", self.target_velocity
        print "Time:", t
        print "Shoot At:", shoot_at
        print "Angle:", angle
        
        return angle
    
    def get_desired_movement_command(self, time_diff, maxspeed):
        # PD Controller stuff to make movement smoother
        #delta_x = self.target[0] - self.x
        #delta_y = self.target[1] - self.y
        #print "Delta:", (delta_x, delta_y)
        
        #target_angle = math.atan2(delta_y, delta_x)
        target_angle = self.estimate_firing_angle()
        current_angle = normalize_angle(self.angle)
        error_angle = normalize_angle(target_angle - current_angle);
        #print "Error:", int(rad2deg(error_angle)), "Target:", int(rad2deg(target_angle)), "Current:", int(rad2deg(current_angle))
        # clamp the speed to -1 to 1 (technically, 0 to 1)
        # Base the speed on the current angle as well
        #target_speed = math.cos(error_angle) * maxspeed
        #current_speed = math.sqrt(math.pow(self.vy, 2) + math.pow(self.vx, 2))
        #error_speed = target_speed - current_speed;
        #print "Error:", int(error_speed), "Target:", int(target_speed), "Current:", int(current_speed)
        
        proportional_gain_angle = 2.25
        #proportional_gain_speed = 1.0
        derivative_gain_angle = 0.5
        #derivative_gain_speed = 0.1
        
        send_angvel = proportional_gain_angle * error_angle + derivative_gain_angle * ((error_angle - self.previous_error_angle) / time_diff)
        #send_speed = proportional_gain_speed * error_speed + derivative_gain_speed * ((error_speed - self.previous_error_speed) / time_diff)
        
        self.previous_error_angle = error_angle
        #self.previous_error_speed = error_speed
        
        '''
        magnitude = math.sqrt(delta_x**2 + delta_y**2)
        if magnitude == 0:
            magnitude = 1
        direction = (delta_x / magnitude, delta_y / magnitude)
        
        #dist((self.vx, self.vy), (0, 0))/time_diff < 1 and math.fabs(error_angle) < math.pi/6: # Did we not move very far, and were we facing the right way?
        if average_grid(self.agent.bel_grid, (self.x + 5 * direction[0] + 400, self.y + 5 * direction[1] + 400), 10) > .8 or (self.x == self.prev_x and self.y == self.prev_y): # Are we reasonably sure we're running into an obstacle right now?
            # If we are hitting an obstacle, send the max angular velocity
            send_angvel = 1
            send_speed = 1
        #    print "true"
        #else:
        #    print "false"
        '''
            
        #return Command(self.index, send_speed, send_angvel, 1)
        return Command(self.index, 0, send_angvel, 1)

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
    numpy.set_printoptions(precision=3,suppress=True)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))
    
    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            new_time = time.time()
            time_diff = new_time - prev_time
            prev_time = new_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()

if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4