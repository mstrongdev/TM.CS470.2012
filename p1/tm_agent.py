#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time

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

class PField(object):
    '''A Potential Field. Parent Class.'''
    # PUBLIC
    def __init__(self, center=(0,0), radius=0, spread=100, strength=1, min_bound=None, max_bound=None, direction=None):
        self.center = center
        self.radius = radius
        self.spread = spread
        self.strength = strength
        self.min_bound = min_bound
        self.max_bound = max_bound
        self.direction = direction
    def update(self, center=None, radius=None, spread=None, strength=None, min_bound=None, max_bound=None, direction=None):
        if center:
            self.center = center
        if radius:
            self.radius = radius
        if spread:
            self.spread = spread
        if strength:
            self.strength = strength
        if min_bound:
            self.min_bound = min_bound
        if max_bound:
            self.max_bound = max_bound
        if direction:
            self.direction = direction
    def get_delta(p):
        return p
    
    # PRIVATE
    def _attract(p):
        d = dist(self.center, p)
        theta = math.atan2((self.center[1]-p[1])/(self.center[0]-p[0]))
        dx,dy = (0,0)
        
        # Position is inside the Goal.
        if d < self.radius: 
            pass
        # Position is outside goal and inside spread.
        elif self.radius <= d and d <= (self.radius+self.spread):
            dx = self.strength * (d-self.radius) * math.cos(theta)
            dy = self.strength * (d-self.radius) * math.sin(theta)
        # Position is outside spread. Delta is at max strength.
        else:
            dx = self.strength * self.spread * math.cos(theta)
            dy = self.strength * self.spread * math.sin(theta)
        
        return (dx,dy)

    def _repulse(p):
        d = dist(self.center, p)
        theta = math.atan2((self.center[1]-p[1])/(self.center[0]-p[0]))
        dx,dy = (0,0)
        
        # Position is inside the Goal.
        if d < self.radius:
            dx = -math.cos(theta) * float("inf")
            dy = -math.sin(theta) * float("inf")
        # Position is outside goal and inside spread.
        elif self.radius <= d and d <= (self.radius+self.spread):
            dx = self.strength * (self.spread + self.radius - d) * math.cos(theta)
            dy = self.strength * (self.spread + self.radius - d) * math.sin(theta)
        # Position is outside spread.
        else:
            pass
        
        return (dx,dy)

    def _tangent(rotation, p):
        pass
    def _random(p):
        pass
    def _boundedUniform(p):
        pass
    def _boundedPerpendicular(p):
        pass

def normalize_angle(angle):
    '''Make any angle be between +/- pi.'''
    angle -= 2 * math.pi * int (angle / (2 * math.pi))
    if angle <= -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle
    
class Agent(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = {tank.index:Tank(bzrc, tank) for tank in self.bzrc.read_mytanks()}

    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        # Get information from the BZRC server
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                self.constants['team']]

        # Reset my set of commands (we don't want to run old commands)
        self.commands = []

        # Decide what to do with each of my tanks
        for bot in mytanks:
            self.tanks[bot.index].update(bot)
            # Figure out the desired potential field magic to give it a vector. Make one up for now
            delta_x = 5
            delta_y = 10
            self.commands.append(self.tanks[bot.index].get_desired_movement_command(delta_x, delta_y, time_diff))

        # Send the commands to the server
        results = self.bzrc.do_commands(self.commands)

    """
    def attack_enemies(self, bot):
        '''Find the closest enemy and chase it, shooting as you go'''
        best_enemy = None
        best_dist = 2 * float(self.constants['worldsize'])
        for enemy in self.enemies:
            if enemy.status != 'alive':
                continue
            dist = math.sqrt((enemy.x - bot.x)**2 + (enemy.y - bot.y)**2)
            if dist < best_dist:
                best_dist = dist
                best_enemy = enemy
        if best_enemy is None:
            command = Command(bot.index, 0, 0, False)
            self.commands.append(command)
        else:
            self.move_to_position(bot, best_enemy.x, best_enemy.y)

    def move_to_position(self, bot, target_x, target_y):
        target_angle = math.atan2(target_y - bot.y,
                target_x - bot.x)
        relative_angle = normalize_angle(target_angle - bot.angle)
        command = Command(bot.index, 1, 2 * relative_angle, True)
        self.commands.append(command)
    """
    
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
    
    def get_desired_movement_command(self, delta_x, delta_y, time_diff):
        target_angle = math.atan2(delta_y, delta_x)
        # clamp the speed to -1 to 1 (technically, 0 to 1)
        target_speed = min(math.sqrt(math.pow(delta_x) + math.pow(delta_y)), 1.0)
        current_angle = self.angle;
        current_speed = math.sqrt(math.pow(self.vy) + math.pow(self.vx))
        error_angle = target_angle - current_angle;
        error_speed = target_speed - current_speed;
        
        proportional_gain_angle = 0.1
        proportional_gain_speed = 0.1
        derivative_gain_angle = 0.1
        derivative_gain_speed = 0.1
        
        send_angle = proportional_gain_angle * error_angle + derivative_gain_angle * ((error_angle - previous_error_angle) / time_diff)
        send_speed = proportional_gain_speed * error_speed + derivative_gain_speed * ((error_speed - previous_error_speed) / time_diff)
        
        self.previous_error_angle = error_angle
        self.previous_error_speed = error_speed
        
        return Command(self.index, send_speed, send_angle, 0)


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

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