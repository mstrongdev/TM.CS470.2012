#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time, Queue, random

class Agent(object):
    
    
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        #self.dumb_queue = Queue.PriorityQueue()
        self.tank_tracker = {}

    def tick(self, time_diff, tick_time):
        '''Some time has passed; decide what to do next'''
        # Get information from the BZRC server
        #self.mytanks = self.bzrc.get_mytanks()

        # Reset my set of commands (we don't want to run old commands)
        #self.commands = []

        # Decide what to do with each of my tanks
        #self.do_dumb_stuff(tick_time)

        # Send the commands to the server
        #results = self.bzrc.do_commands(self.commands)

    def do_dumb_stuff(self, tick_time):
        turn_speed = math.pi / 6 # 30 degrees per second
        if not self.tank_tracker: # Initialize
            for bot in self.mytanks:
                move_duration = random.random() * 5 + 3
                turn_angle = math.radians(60 + random.random() * 10)
                turn_duration = turn_angle / turn_speed
                shoot_time = random.random() + 1.5
                
                self.tank_tracker[bot.index] = [move_duration, move_duration+turn_duration, shoot_time]
                self.commands.append(Command(bot.index,1,0,0))
        
        for bot in self.mytanks:
            if bot.status == "alive": # Delay commands for dead tank until it respawns
                self.tank_tracker[bot.index][0] -= tick_time
                self.tank_tracker[bot.index][1] -= tick_time
                self.tank_tracker[bot.index][2] -= tick_time

            if self.tank_tracker[bot.index][0] < 0:
                self.commands.append(Command(bot.index, 0,turn_speed,0))
                self.tank_tracker[bot.index][0] = random.random() * 5 + 3 + self.tank_tracker[bot.index][1]
            if self.tank_tracker[bot.index][1] < 0:
                self.commands.append(Command(bot.index, 1, 0, 0))
                self.tank_tracker[bot.index][1] = math.radians(60 + random.random() * 10)/turn_speed + self.tank_tracker[bot.index][0]
            if self.tank_tracker[bot.index][2] < 0:
                self.commands.append(Command(bot.index, None, None, 1))
                self.tank_tracker[bot.index][2] = random.random() + 1.5

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

    start_time = time.time()
    prev_time = start_time
    tick_time = 0.0

    # Run the agent
    try:
        while True:
            now = time.time()
            time_diff = now - start_time
            tick_time = now - prev_time
            prev_time = now
            agent.tick(time_diff, tick_time)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4