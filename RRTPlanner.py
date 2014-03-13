import IPython
import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # At most 5000 nodes
        count = 0
        while count < 5000:
            count = count+1

            # Add a new node
            foundNext = False
            while foundNext == False:
                randConfig = self.planning_env.GenerateRandomConfiguration();

                #find the closest vertex to this configuration
                parentIndex, vertex = tree.GetNearestVertex(randConfig)

                # extend needs to move along the direction towards the new
                # configuration, but can't run into any obstacles
                randConfig = self.planning_env.Extend(vertex,randConfig)

                #check if a collision occurs moving from the parent to child
                if (self.planning_env.LineCollides(vertex,randConfig) == False):
                    foundNext = True


            #add this new configuration to the tree
            newIndex = tree.AddVertex(randConfig)
            #add an edge between the vertex and its nearest neighbor
            tree.AddEdge(newIndex, parentIndex)

            if self.planning_env.RadiusCollision(randConfig, goal_config, GOAL_RADIUS):
                currentState = 'goalFound'
                goalNode = nodes[len(nodes)-1]

            if count%100 == 0:
                print("node: " + str(count))


        plan.append(start_config)
        plan.append(goal_config)

        return plan
