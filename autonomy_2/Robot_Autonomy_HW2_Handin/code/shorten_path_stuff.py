import numpy as np
import matplotlib.pyplot as pl
import random
import time 

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot

    def ShortenPath(self, path):

        len_path = len(path)

        start_time = time.time()

        while (True):

            # Randomly select 2 points from the list of points on the shortest path
            q1_,q2_ = np.random.choice(len_path-1, 2)

            q1,q2 = path[q1_], path[q2_]

            '''
            We now need to see if we are able to connect the points together and create a
            new sample point between the 2. Let us call this sample point qi. The formula
            for qi is given by:
            qi = (q2 - q1)*i + q1 where i is a float that lies in the range [0,1] 
            '''

            # Default condition is connect = 2,
            # If points can't be connected for shortening, connect = 0. If points can be connected for shortening, connect = 1
            connect = 2

            i = 0

            for i in range(0, 1, 0.01):
                
                qi = np.linalg.norm(q2 - q1)* i + q1

                self.robot.SetActiveDOFValues(qi)
                
                # If a collision is imminent, then you must not consider shortening from these 2 points
                if (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(self.robot) ):

                    connect = 0
                    
            # If no collision is imminent, go for it
            connect = 1

            if (connect == 1):

                for j in range(q1_ + 1, q2_, 1):

                    path = np.delete(path, j, 0)
                    

            current_time = time.time()
            if (current_time - start_time >= 5.0):
                break

        return path
