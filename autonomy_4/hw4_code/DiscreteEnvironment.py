import numpy as np

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):
        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = np.array(lower_limits)
        self.upper_limits = np.array(upper_limits)

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension * [0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil(
                (self.upper_limits[idx] - self.lower_limits[idx]) / float(self.resolution[idx])).astype(int)


    def ConfigurationToNodeId(self, config):
        # This function maps a node configuration in full configuration
        # space to a node in discrete space

        grid_coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(grid_coord)
        return node_id


    def NodeIdToConfiguration(self, nid):
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space

        grid_coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(grid_coord)
        return config
    def ConfigurationToGridCoord2(self, config):

            # TODO:
            # This function maps a configuration in the full configuration space
            # to a grid coordinate in discrete space
            #
            # input: config = [0.26, 0.64]
            # output = [1, 3]
            # resolution = 0.1
            # lower_limits = [-5.0,-5.0]
            # Upper_limits = [5.0,5.0]
            # cell num
            coord = [0] * self.dimension
            for i in range(self.dimension):
            	coord[i] = numpy.ceil((config[i]-self.lower_limits[i])/self.resolution[i]) - 1
            	if coord[i]==-1:
            		coord[i]=0
            if any(n < 0 for n in coord):
                print("coord: {}, n: {}".format(coord, n))
            return coord

    def ConfigurationToGridCoord(self, config):
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space

        coord = [0] * self.dimension
        for i in range(self.dimension):
            coord[i] = (config[i] - self.lower_limits[i]) // self.resolution[i]
        coord = np.clip(coord, 0, np.array(self.num_cells) - 1)
        return coord


    def GridCoordToConfiguration(self, coord):
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space

        config = [0] * self.dimension
        for i in range(self.dimension):
            config[i] = coord[i] * self.resolution[i] + self.resolution[i] / 2 + self.lower_limits[i]
        config = np.clip(np.array(config), self.lower_limits + 0.1, self.upper_limits - 0.1)
        return config


    def GridCoordToNodeId(self, coord):
        # This function maps a grid coordinate to the associated
        # node id
        node_id = np.ravel_multi_index(np.array(coord).astype(int), self.num_cells)
        return node_id


    def NodeIdToGridCoord(self, node_id):
        coord = [0] * self.dimension
        coord = np.unravel_index(node_id, self.num_cells)
        if min(self.num_cells - np.array(coord)) < 0:
            print("NodeId2Grid dim < 0")
            raise
        return coord
