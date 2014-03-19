import numpy
import pylab as pl
import random
import time
import argparse, numpy, openravepy, time


class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]
 
        self.env = self.robot.GetEnv()
        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        #self.env.CollisionCheck(self.rbot,self.table)
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        xPoint = random.uniform(lower_limits[0], upper_limits[0])
        yPoint = random.uniform(lower_limits[1], upper_limits[1])

        detection = True 
        while (detection == True):
            xPoint = random.uniform(lower_limits[0], upper_limits[0])
            yPoint = random.uniform(lower_limits[1], upper_limits[1])
            detection = self.Collides([xPoint,yPoint], 0)
        #print("x = "+str(xPoint) + " y = "+str(yPoint));
        config = [xPoint, yPoint]
        #pl.plot(config, 'gx')
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        x1 = start_config[0]
        y1 = start_config[1]
        x2 = end_config[0]
        y2 = end_config[1]

        dist  = numpy.sqrt((x1-x2)**2 + (y1-y2)**2) 
        
        return dist

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        a = start_config
        b = end_config
        points = []
        for i in range(1,101):
            x_step = (b[0]-a[0])/100
            y_step = (b[1]-a[1])/100
            points.append([x_step*i+a[0],y_step*i+a[1]])
        collision_index = 99
        for i in range(0,100):
            if (self.Collides(points[i], 0)):
                collision_index = i
                break
        if collision_index == 0:
            return None
        elif collision_index < 99:
            return numpy.array(points[i-1])
        else:
            return numpy.array(points[len(points)-1])        

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        initial_time = time.time()
        removed = 1 
        times = 1    
        while removed == 1 and times < timeout:
            
            removed = 0
            idx =1 
            while idx < len(path)-1:
                n1 = path[idx-1]
                n2 = path[idx+1]
                if(self.Bs(n1,n2)):
                    removed = 1
                    path[idx] = None
                    path.remove(None)
                else:
                    idx = idx + 1
                get_time = time.time()
                times = get_time - initial_time

        return path



    def Bs(self, start_config, end_config):
        
        a = start_config
        b = end_config
        points = []
        for i in range(1,101):
            x_step = (b[0]-a[0])/100
            y_step = (b[1]-a[1])/100
            points.append([x_step*i+a[0],y_step*i+a[1]])
        collision_index = 99
        for i in range(0,100):
            if (self.Collides(points[i], 0)):
                collision_index = i
                break
        if collision_index < 99:
            return False
        else:
            return True


    def Collides(self, point, theta):
        # Show all obstacles in environment
        #if(env.CheckCollision(robot1,robot2)
        x = point[0]
        y = point[1]
        transform = self.robot.GetTransform()
        transform[0][3] = x
        transform[1][3] = y
        self.robot.SetTransform(transform)
        if self.env.CheckCollision(self.robot,self.table) == True:
            return True
        else:
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

    def ComputePathLength(self, plan):      
        dist = 0 
        for i in range(0,len(plan)-1):
            x = self.ComputeDistance(plan[i],plan[i+1])
            dist = dist + x       
        return dist
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()


