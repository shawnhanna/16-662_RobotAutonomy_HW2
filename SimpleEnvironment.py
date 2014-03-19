import numpy
import pylab as pl
import random
import IPython
import math

class SimpleEnvironment(object):

    def __init__(self, simpleRobot):
        self.robot = simpleRobot.robot
        self.simpleBot = simpleRobot

        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

        # max movement from a known node to a new node
        self.epsilon = 0.1 #meters


    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p


    def GenerateRandomConfiguration(self):
        lower_limits, upper_limits = self.boundary_limits

        xPoint = random.uniform(lower_limits[0], upper_limits[0])
        yPoint = random.uniform(lower_limits[1], upper_limits[1])

        if (self.Collides([xPoint,yPoint])):
            print("Collision detected")

        print("x = "+str(xPoint) + " y = "+str(yPoint));

        config = [xPoint, yPoint]
        # pl.plot(config, 'gx')

        return numpy.array(config)


    def ComputeDistance(self, start_config, end_config):
        xdist = end_config[0] - start_config[0]
        ydist = end_config[1] - start_config[1]
        return math.sqrt(xdist * xdist + ydist * ydist)


    def Extend(self, start_config, end_config):

        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        if self.ComputeDistance(start_config,end_config) < self.epsilon:
            return end_config
        else:
            theta = math.atan2(end_config[1]-start_config[1],end_config[0]-start_config[0])
            return start_config[0] + self.epsilon*math.cos(theta), start_config[1] + self.epsilon*math.sin(theta)


        travel_limit = 0.01 # maximum distance allowed to travel towards goal
        num_checks = 10 # number of collision checks along the way
        dist = self.ComputeDistance(start_config, end_config)

        travel_dist = min(travel_limit, dist)

        x0 = start_config[0]
        y0 = start_config[1]
        x1 = end_config[0]
        y1 = end_config[1]
        xg = x0 + ( (travel_dist/dist)*(x1-x0) ) # xg and yg limited by travel_limit
        yg = y0 + ( (travel_dist/dist)*(y1-y0) )

        # incrementally step along line from start to goal, checking for collisions
        for i in range(0, num_checks-1):
            config[0] = x0 + ( (i/(num_checks-1)) * (xg-x0) )
            config[1] = y0 + ( (i/(num_checks-1)) * (yg-y0) )
            if self.Collides(config):
                return None


    def ShortenPath(self, path, timeout=5.0):
        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path


    def Collides(self, point, theta):
        T = self.robot.GetTransform()
        Tnew = T
        Tnew[[0,1],3] = point
        env = self.robot.GetEnv()

        # move robot to potential point, return collision between robot and table
        self.robot.SetTransform(Tnew)
        return env.CheckCollision(self.robot, env.GetKinBody('conference_table'))




    # Returns true if the point specified collides with an object in the space
    def Collides(self, point):
        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            tl = [bb.pos()[0] - bb.extents()[0], bb.pos()[1]-bb.extents()[1]]
            br = [bb.pos()[0] + bb.extents()[0], bb.pos()[1]+bb.extents()[1]]

            x = point[0]
            y = point[1]

            if ((x <= br[0]) and (x >= tl[0]) and (y <= tl[1]) and (y >= br[1])):
                return True

        return False


    # Returns true if the line segment  specified collides with an object in the space
    def LineCollides(self, start_point, end_point):
        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            tl = [bb.pos()[0] - bb.extents()[0], bb.pos()[1]-bb.extents()[1]]
            br = [bb.pos()[0] + bb.extents()[0], bb.pos()[1]+bb.extents()[1]]

            x = 0
            y = 0

            if ((x <= br[0]) and (x >= tl[0]) and (y <= tl[1]) and (y >= br[1])):
                return True

        return False

    # returns true if the points p1 & p2 are within radius specified
    def RadiusCollision(p1, p2, radius):
        distance = self.ComputeDistance(p1,p2)
        if (distance <= radius):
            return True
        return False


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')

        pl.ion()
        pl.show()


    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
