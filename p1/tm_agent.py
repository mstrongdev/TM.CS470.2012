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
    
def midpoint(pt1, pt2):
    return ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2)
    
def add(vector1, vector2):
    return (vector1[0] + vector2[0], vector1[1] + vector2[1])
    
def average(a, b, c, d):
    return (a + b + c + d) / 4

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
            
    def get_force(self, position):
        return (0,0)
    
    # PRIVATE
    def _attract(self, p, strength=None):
        d = dist(self.center, p)
        theta = math.atan2(self.center[1]-p[1], self.center[0]-p[0])
        scale = strength if strength else self.strength
        dx,dy = (0,0)
        
        # Position is inside the Goal.
        if d < self.radius: 
            pass
        # Position is outside goal and inside spread.
        elif self.radius <= d and d <= (self.radius+self.spread):
            dx = scale * (d-self.radius) * math.cos(theta)
            dy = scale * (d-self.radius) * math.sin(theta)
        # Position is outside spread. Delta is at max strength.
        else:
            dx = scale * self.spread * math.cos(theta)
            dy = scale * self.spread * math.sin(theta)
        
        return (dx,dy)
        
    def _gravity(self, p, strength=None):
        d = dist(self.center, p)
        theta = math.atan2(self.center[1]-p[1], self.center[0]-p[0])
        scale = strength if strength else self.strength
        dx,dy = (0, 0)
        
        # doesn't use radius or spread - just strength as a scale on the distance squared
        if (d > 1):
            dx = min(scale * (1 / d), 25) * math.cos(theta)
            dy = min(scale * (1 / d), 25) * math.sin(theta)
        else:
            # If right on top of it, don't give a super huge movement force.
            pass
        
        return (dx, dy)

    def _repulse(self, p, strength=None, theta_offset=0):
        d = dist(self.center, p)
        theta = math.atan2(self.center[1]-p[1], self.center[0]-p[0]) + theta_offset
        scale = strength if strength else self.strength
        dx,dy = (0,0)
        
        # Position is inside the Goal.
        if d < self.radius:
            dx = -math.cos(theta) * 5 #float("inf")
            dy = -math.sin(theta) * 5 #float("inf")
        # Position is outside goal and inside spread.
        elif self.radius <= d and d <= (self.radius+self.spread):
            dx = -scale * (self.spread + self.radius - d) * math.cos(theta)
            dy = -scale * (self.spread + self.radius - d) * math.sin(theta)
        # Position is outside spread.
        else:
            pass
        
        return (dx,dy)

    def _tangent(self, p, clockwise, strength=None):
        theta_direction = math.pi/2 if clockwise else -math.pi/2  # TODO: Does this need to be flipped?
        return self._repulse(p, strength, theta_direction)
    
    def _random(self, p, strength=None, time=0):
        scale = strength if strength else self.strength
        
        seed = p[0]*41648 + p[1]*915
        theta = random.random(seed+time)*2*math.pi - math.pi
        seed = p[0]*743 + p[1]*13743
        r_magnitude = random.random(seed+time) * scale
        
        dx = magnitude * math.cos(theta)
        dy = magnitude * math.sin(theta)
        
        return (dx,dy)

    def _boundedUniform(self, p):
        pass
    def _boundedPerpendicular(self, p):
        pass

class ObstacleField(PField):
    
    def __init__(self, obstacle):
        new_center = (average(obstacle[0][0], obstacle[1][0], obstacle[2][0], obstacle[3][0]), average(obstacle[0][1], obstacle[1][1], obstacle[2][1], obstacle[3][1]))
        new_radius = min(dist(new_center, obstacle[0]), dist(new_center, obstacle[1]), dist(new_center, obstacle[2]), dist(new_center, obstacle[3]),
            dist(new_center, midpoint(obstacle[0], obstacle[1])), dist(new_center, midpoint(obstacle[1], obstacle[2])),
            dist(new_center, midpoint(obstacle[2], obstacle[3])), dist(new_center, midpoint(obstacle[3], obstacle[0])))
        new_spread = max(dist(new_center, obstacle[0]), dist(new_center, obstacle[1]), dist(new_center, obstacle[2]), dist(new_center, obstacle[3])) - new_radius + 50
        self.update(
            center = new_center,
            radius = new_radius,
            spread = new_spread,
            strength = .1)
    
    def get_force(self, position):
        return add(self._repulse(position), self._tangent(position, True))

class FlagField(PField):
    
    def __init__(self, flag, agent):
        self.update(
            center = (flag.x, flag.y),
            radius = float(agent.constants['flagradius']),
            spread = 50,
            strength = .1)
            
    def get_force(self, position):
        return add(self._gravity(position, 1500), self._attract(position))
        
class BaseField(PField):
    
    def __init__(self, base):
        new_center = (average(base.corner1_x, base.corner2_x, base.corner3_x, base.corner4_x), average(base.corner1_y, base.corner2_y, base.corner3_y, base.corner4_y))
        corner1 = (base.corner1_x, base.corner1_y)
        corner2 = (base.corner2_x, base.corner2_y)
        corner3 = (base.corner3_x, base.corner3_y)
        corner4 = (base.corner4_x, base.corner4_y)
        new_radius = min(dist(new_center, corner1), dist(new_center, corner2), dist(new_center, corner3), dist(new_center, corner4),
            dist(new_center, midpoint(corner1, corner2)), dist(new_center, midpoint(corner2, corner3)),
            dist(new_center, midpoint(corner3, corner4)), dist(new_center, midpoint(corner4, corner1)))
        new_spread = max(dist(new_center, corner1), dist(new_center, corner2), dist(new_center, corner3), dist(new_center, corner4)) - new_radius
        self.update(
            center = new_center,
            radius = new_radius,
            spread = new_spread,
            strength = 50)
            
    def get_force(self, position):
        return self._attract(position)

class Agent(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = {tank.index:Tank(bzrc, tank) for tank in self.bzrc.get_mytanks()}
        
        # Write the initial potential fields
        fields = []
        for obstacle in self.bzrc.get_obstacles():
            fields.append(ObstacleField(obstacle))
        for flag in self.bzrc.get_flags():
            if (flag.color != self.constants['team']):
                fields.append(FlagField(flag, self))
        for base in self.bzrc.get_bases():
            if (base.color == self.constants['team']):
                #fields.append(BaseField(base))
                pass
        
        with open("obstacle_fields.gpi", 'w+') as out:
            # header
            out.write("set title \"Potential Fields\"\nset xrange [-400.0 : 400.0]\nset yrange [-400.0 : 400.0]\nunset key\nset size square\nset terminal wxt size 1600,1600\nset term png\nset output \"output.png\"\n\n")
            
            # write the body of the fields
            for x in range(-390, 390, 20):
                for y in range(-390, 390, 20):
                    vector = (0, 0)
                    for field in fields:
                        vector = add(vector, field.get_force((x, y)))
                    if (vector[0] != float("inf") and vector[1] != float("-inf")):
                        out.write("set arrow from {0}, {1} to {2}, {3}\n".format(x - vector[0]/2, y - vector[1]/2, x + vector[0]/2, y + vector[1]/2))
                        #pass
                    
            # start plot
            out.write("plot '-' with lines\n0 0 1 1\n")
                
            # end plot
            out.write("e\n");
        

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

        # As obstacles can apparently change with visibility, load them on each tick
        fields = []
        for obstacle in self.bzrc.get_obstacles():
            fields.append(ObstacleField(obstacle))
        for flag in self.flags:
            if (flag.color != self.constants['team']):
                fields.append(FlagField(flag, self))
        
        # Decide what to do with each of my tanks
        for bot in mytanks:
            tank = self.tanks[bot.index]
            tank.update(bot)
            # Figure out the desired potential field magic to give it a vector. Make one up for now
            delta_x = 5
            delta_y = 10
            movement = (delta_x, delta_y)
            
            tank_pos = (tank.x, tank.y)
            for field in fields:
                movement = add(movement, field.get_force(tank_pos))
            if tank.flag != '-':
                for base in self.bzrc.get_bases():
                    if base.color == self.constants['team']:
                        movement = add(movement, BaseField(base).get_force(tank_pos))
            
            self.commands.append(tank.get_desired_movement_command(movement, time_diff))

        # Send the commands to the server
        results = self.bzrc.do_commands(self.commands)

    '''
    def attack_enemies(self, bot):
        #Find the closest enemy and chase it, shooting as you go
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
    '''
    
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
    
    def get_desired_movement_command(self, movement_vector, time_diff):
        delta_x, delta_y = movement_vector
        
        target_angle = math.atan2(delta_y, delta_x)
        # clamp the speed to -1 to 1 (technically, 0 to 1)
        target_speed = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
        current_angle = self.angle;
        current_speed = math.sqrt(math.pow(self.vy, 2) + math.pow(self.vx, 2))
        error_angle = target_angle - current_angle;
        error_speed = target_speed - current_speed;
        
        proportional_gain_angle = 1
        proportional_gain_speed = 1
        derivative_gain_angle = 0.1
        derivative_gain_speed = 0.1
        
        send_angle = proportional_gain_angle * error_angle + derivative_gain_angle * ((error_angle - self.previous_error_angle) / time_diff)
        send_speed = proportional_gain_speed * error_speed + derivative_gain_speed * ((error_speed - self.previous_error_speed) / time_diff)
        
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
        
    print "Running Tyler & Morgan's Super Smart Flag Capturer"

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