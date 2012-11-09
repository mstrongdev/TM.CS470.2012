#!/usr/bin/python -tt

from bzrc import BZRC, Command
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
    
def average(l):
    return reduce(lambda x, y: x + y, l) / len(l)

class Agent(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = {tank.index:Tank(bzrc, tank) for tank in self.bzrc.get_mytanks()}
        self.bel_grid = [[]]
        
    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        
        # ALGORITHM
        # 
        
        
        # Reset my set of commands (we don't want to run old commands)
        self.commands = []
        
        # Decide what to do with each of my tanks
        for bot in mytanks:
            tank = self.tanks[bot.index]
            tank.update(bot)
            
            self.commands.append(tank.get_desired_movement_command(movement, time_diff, tank.flag != '-'))

        # Send the commands to the server
        results = self.bzrc.do_commands(self.commands)
    
class Tank(object):
    
    def __init__(self, bzrc, tank):
        self.bzrc = bzrc
        self.previous_error_angle = 0
        self.previous_error_speed = 0
        self.update(tank)
    
    def update(self, tank):
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
    
    def get_desired_movement_command(self, movement_vector, time_diff, has_flag):
        # PD Controller stuff to make movement smoother
        delta_x, delta_y = movement_vector
        
        target_angle = math.atan2(delta_y, delta_x)
        # clamp the speed to -1 to 1 (technically, 0 to 1)
        target_speed = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
        current_angle = normalize_angle(self.angle);
        current_speed = math.sqrt(math.pow(self.vy, 2) + math.pow(self.vx, 2))
        error_angle = normalize_angle(target_angle - current_angle);
        error_speed = target_speed - current_speed;
        
        proportional_gain_angle = 1.25
        proportional_gain_speed = 0.1
        derivative_gain_angle = 0.1
        derivative_gain_speed = 0.1
        
        send_angvel = proportional_gain_angle * error_angle + derivative_gain_angle * ((error_angle - self.previous_error_angle) / time_diff)
        send_speed = proportional_gain_speed * error_speed + derivative_gain_speed * ((error_speed - self.previous_error_speed) / time_diff)
        
        self.previous_error_angle = error_angle
        self.previous_error_speed = error_speed
        
        return Command(self.index, send_speed, send_angvel, 1 if self.shots_avail and (random.random() * 100 < 3 or has_flag) else 0) # Shoot sporadically

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