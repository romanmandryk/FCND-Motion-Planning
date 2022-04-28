import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

COLLIDERS_CSV = 'colliders.csv'


def reverse_lat_lng(pos):
    return (pos[1], pos[0], pos[2])


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):
    pruned_path = path.copy()
    i = 0
    while i < len(pruned_path) - 2:
        if collinearity_check(point(pruned_path[i]), point(pruned_path[i + 1]), point(pruned_path[i + 2])):
            pruned_path.pop(i + 1)
        else:
            i += 1
    return pruned_path


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def read_home_location(self):
        f = open(COLLIDERS_CSV)
        pos_str = f.readline().split(',')
        pos = []
        for str in pos_str:
            pos.append(float(str[5:]))
        return np.array(pos)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        # initial lat0 37.792480 lon0 -122.397450
        #GOAL_GLOBAL = (37.792880, -122.397450, TARGET_ALTITUDE)
        #GOAL_GLOBAL = (37.792580, -122.398950, TARGET_ALTITUDE)
        GOAL_GLOBAL = (37.794780, -122.399450, TARGET_ALTITUDE)
        # GOAL_GLOBAL = (37.795480, -122.401950, TARGET_ALTITUDE) # failing in AttributeError: 'int' object has no attribute 'time' drone.py:117
        # GOAL_GLOBAL = (37.789980, -122.393550, TARGET_ALTITUDE) # failing at ConnectionResetError: [Errno 54] Connection reset by peer

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        home_pos = self.read_home_location()

        # set home position to (lon0, lat0, 0)
        self.set_home_position(home_pos[1], home_pos[0], 0)

        # retrieve current global position
        # convert to current local position using global_to_local()
        local_start = global_to_local(self.global_position, self.global_home)
        print('local_start', local_start, self.local_position)
        # NOTE: it seems I could just use self.local_position

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(COLLIDERS_CSV, delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        local_goal = global_to_local(reverse_lat_lng(GOAL_GLOBAL), self.global_home)
        self.target_position[0] = local_goal[0]
        self.target_position[1] = local_goal[1]
        # convert start position to current position rather than map center
        grid_start = (int(local_start[0]) - north_offset, int(local_start[1]) - east_offset)
        # Set goal as some arbitrary position on the grid
        # adapt to set goal as latitude / longitude position and convert
        grid_goal = (int(local_goal[0]) - north_offset, int(local_goal[1]) - east_offset)
        # Run A* to find a path from start to goal
        # add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        if grid[grid_goal[0], grid_goal[1]] == 1:
            print("GOAL inside the obstacle! Change goal position.")
            exit(-1)
        if grid[grid_start[0], grid_start[1]] == 1:
            print("start inside the obstacle! Change start position.")
            exit(-1)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = prune_path(path)
        print('lengths of paths (initial,pruned)', len(path), len(pruned_path))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
