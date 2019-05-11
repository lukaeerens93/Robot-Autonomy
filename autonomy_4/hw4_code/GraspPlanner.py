import logging, numpy, openravepy
import time
import IPython
import pickle
import numpy as np
class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.env = self.robot.GetEnv()
        self.manip = self.robot.GetActiveManipulator()


    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        #load model of base
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        if not self.irmodel.load():
            self.irmodel.autogenerate()
        print "models are loaded"
        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################
        print "start calculate grasp and base!!!!!!"
        self.graspindices=self.gmodel.graspindices
        self.grasps=self.gmodel.grasps
        self.order_grasps()
        Tgrasp = self.gmodel.getGlobalGraspTransform(self.grasps_ordered[10],collisionfree=True)
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)
        # pickle.dump([ Tgrasp,densityfn,samplerfn,bounds ], open( "save.p", "wb" ) )

        # Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame
        # equals manip.GetTransform() in the goal state
        print "got irmodel {}".format(Tgrasp)
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = 10000
        N = 5
        print "start time {}".format(starttime)
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                            print "q: {}".format(q)
                        elif self.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1
        print 'showing {} results'.format(N)
        for ind,goal in enumerate(goals):
            raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,pose,values = goal
            self.robot.SetTransform(pose)
            self.robot.SetDOFValues(values)
            base_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
            grasp_config = np.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        print "found goals!"
        IPython.embed()
        theta = -numpy.pi/4.
        start_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -1.25],
                                  [numpy.sin(theta),  numpy.cos(theta), 0,  0.82],
                                  [0.              ,  0.              , 1,  0.  ],
                                  [0.              ,  0.              , 0,  1.  ]])
        right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
        left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
        self.robot.SetTransform(start_pose)
        self.robot.SetActiveDOFValues(left_relaxed)

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):
        print "plan to grasp!"
        IPython.embed()

        # Next select a pose for the base and an associated ik for the arm
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())

        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)
        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        # start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        # self.robot.SetTransform(start_pose)
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        # start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        # self.robot.SetDOFValues(start_config)

        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        IPython.embed()
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()

    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy()  # you should change the order of self.grasps_ordered

        # Weights for each score
        weights = np.array([.5, .5])

        # Get the Score of score and add it performance
        performance = np.zeros(shape=(np.shape(self.grasps_ordered)[0], weights.shape[0]), dtype=np.float64)

        for index, grasp in enumerate(self.grasps_ordered):
            performance[index, :] = np.array(self.eval_grasp(grasp), dtype=np.float64)

        # Normalize the performance
        performance = performance / sum(performance, 0)

        # Calculated the weighted average
        for index, grasp in enumerate(self.grasps_ordered):
            self.grasps_ordered[index][self.graspindices.get('performance')] = np.dot(weights, performance[index, :])

        # sort!
        order = np.argsort(self.grasps_ordered[:, self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
        with self.robot:
            # Contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp, translate=True,
                                                                               forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3, 3]
                # for each contact
                G = np.zeros(shape=(6, contacts.shape[0]), dtype=np.float64)  # the wrench matrix

                for index, c in enumerate(contacts):
                    pos = c[0:3] - obj_position
                    dir = -c[3:]  # this is already a unit vector

                    # Fill the G matrix
                    G[:, index] = np.hstack((dir, np.cross(pos, dir)))

                    # TODO use G to compute scrores as discussed in class

                    # Calculate the singular values of G
                    [_, S, _] = np.linalg.svd(G)

                    # M1 -- Return the minimum singular value
                    score1 = S[-1]

                    # M2 -- Volume metric
                    score2 = np.abs(np.linalg.det(np.dot(G, np.transpose(G))))
                    if np.isnan(score2):
                        print "Not defined"

                return [score1, score2]

            except openravepy.planning_error, e:
                # you get here if there is a failure in planning
                # example: if the hand is already intersecting the object at the initial position/orientation
                return 0.00  # TODO you may want to change this, ANS -- I don't want to change it because it's correct :)

                # heres an interface in case you want to manipulate things more specifically
                # NOTE for this assignment, your solutions cannot make use of graspingnoise
                #      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
                #      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
                #      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
                #      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
                #      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
                #            direction             = grasp[self.graspindices.get('igraspdir')], \
                #            roll                  = grasp[self.graspindices.get('igrasproll')], \
                #            position              = grasp[self.graspindices.get('igrasppos')], \
                #            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
                #            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
                #            target                = self.target_kinbody, \
                #            graspingnoise         = 0.0, \
                #            forceclosure          = True, \
                #            execute               = False, \
                #            outputfinal           = True, \
                #            translationstepmult   = None, \
                #            finestep              = None )

    # given grasp_in, create a new grasp which is altered randomly
    # you can see the current position and direction of the grasp by:
    # grasp[self.graspindices.get('igrasppos')]
    # grasp[self.graspindices.get('igraspdir')]

    def sample_random_grasp(self, grasp_in):
        grasp = grasp_in.copy()

        # Sample random position
        RAND_DIST_SIGMA = 0.0001  # TODO you may want to change this
        # Set a random position
        grasp[self.graspindices['igrasppos']] = np.random.normal(grasp[self.graspindices['igrasppos']], RAND_DIST_SIGMA,
                                                                 (1, 3))

        # Sample random orientation
        RAND_ANGLE_SIGMA = np.pi / 500  # TODO you may want to change this
        grasp[self.graspindices['igraspdir']] = np.random.normal(grasp[self.graspindices['igraspdir']],
                                                                 RAND_ANGLE_SIGMA, (1, 3))  # 43, 44, 45
        grasp[self.graspindices['igrasproll']] = np.random.normal(grasp[self.graspindices['igrasproll']],
                                                                  RAND_ANGLE_SIGMA, 1)  # Don't remember

        return grasp

    # displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
            with self.gmodel.GripperVisibility(self.gmodel.manip):
                time.sleep(0.1)  # let viewer update?
                try:
                    with self.env:
                        contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp, translate=True,
                                                                                       forceclosure=True)
                        # if mindist == 0:
                        #  print 'grasp is not in force closure!'
                        contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                        self.gmodel.robot.GetController().Reset(0)
                        self.gmodel.robot.SetDOFValues(finalconfig[0])
                        self.gmodel.robot.SetTransform(finalconfig[1])
                        self.env.UpdatePublishedBodies()
                        time.sleep(delay)
                except openravepy.planning_error, e:
                    print 'bad grasp!', e
