import numpy as np
import openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb
import time

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -np.pi], [5., 5., np.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [np.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
            #Feed forward the velocities
            # print(xdot, ydot)
            if timecount + stepsize > dt:
                stepsize = dt - timecount

            config = config + stepsize*np.array([xdot, ydot, tdot]).T
            # print(config)
            if config[2] > np.pi:
                config[2] -= 2.*np.pi
            if config[2] < -np.pi:
                config[2] += 2.*np.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        for action in actions:
            print(idx*self.resolution[2]+lower_limits[2])
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            print(action.control.ul, action.control.ur)
            print(xpoints, ypoints)
            pl.plot(xpoints, ypoints, 'k')

        pl.ion()
        pl.show()


    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # self.discrete_env.num_cells shape = [40,40,8]
        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)
            print(start_config)
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            w_left = np.array([-1,-1,1,1])
            w_right = np.array([-1,1,-1,1])
            duration = 1

            for i in range(w_left.size):
                control = Control(w_left[i],w_right[i],duration)
                footprint = self.GenerateFootprintFromControl(start_config,control)
                self.actions[idx].append(Action(control,footprint))


    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        lower_limits, upper_limits = self.boundary_limits
        cur_config = self.discrete_env.NodeIdToConfiguration(node_id)
        orig_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        # print("shape")
        # print(len(self.actions))

        # there are 8 orientations around the robot, choose the current orientation
        cur_orient_actions = self.actions[orig_coord[2]]
        # Given the orientation, find the corresponding control and footprints, add the footprint to the orignal config to get the new config
        for action in cur_orient_actions:
            new_config = np.copy(cur_config)
            valid_action = True
            for footprint in action.footprint:
                new_config[0] = cur_config[0] + footprint[0]
                new_config[1] = cur_config[1] + footprint[1]
                new_config[2] = footprint[2]

                # check weather the new_config is within the boundary
                if new_config[0] >= upper_limits[0] or new_config[0] <= lower_limits[0]:
                    valid_action = False
                    break
                if new_config[1] >= upper_limits[1] or new_config[1] <= lower_limits[1]:
                    valid_action = False
                    break
                if self.CheckCollision(new_config):
                    valid_action = False
                    break

            if valid_action:
                successors.append((self.discrete_env.ConfigurationToNodeId(new_config),action))
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = np.linalg.norm(start_config-end_config)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids
        heurist_cost = self.ComputeDistance(start_id, goal_id)
        return heurist_cost

    def CheckCollision(self, config):
        collide = False
        with self.robot:
            robot_tf = np.identity(4)
            robot_tf[0, 3] = config[0]
            robot_tf[1, 3] = config[1]
            self.robot.SetTransform(robot_tf)
            collide = self.robot.GetEnv().CheckCollision(self.robot)
        return collide
