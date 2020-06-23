#!/usr/bin/env python
# coding: utf-8

# In[1]:


import time, timeit, random, pygame, sys
from math import *
import numpy as np


XDIM = 1000 #window length 
YDIM = 1200 #window breadth

WINSIZE = [XDIM, YDIM]
EPSILON = 7.0 #threshold 
NUMNODES = 10000
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0 #incremental distance
GAME_LEVEL = 1

RANDOM_COUNT = 10000

pygame.init()
fpsClock = pygame.time.Clock()


#screen parameters and variable
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('Q-learning')
white = 255, 240, 200
black = 20, 20, 40
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,255,255


#parameters and variables for Q equation
gamma=0.9  #discount factor 
loss=0.99  
epochs=1200 #number of iterations


class __Q_table:
    
    def __init__(self):
        
        self.rows = 104
        self.cols = 148
        
    def dist(p0, p1):      
        return sqrt((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2)
        
    def initialise_table(Current_point, Goal_point_table):
        
        global Q_table
        Q_table = np.zeros((self.rows, self.cols))
        
        for i in range(0,1000,5):
            for j in range(0, 1200, 5):
                
                if collides((i, j)) == False:
                    Goal_point_table[0] = (Goal_point_table[0]//5)*5
                    Goal_point_table[1] = (Goal_point_table[1]//5)*5
                    
                if (dist(Current_point, Goal_point_table)>= dist([i,j], Goal_point_table)):
                    Q = 500/(dist([i, j])+10)
                    x=j//5
                    y=i//5
                    Q_table[x][y]+=Q
                    
                    return Q
                
            return 0
        
        
    def reward(Current_point, goal_point):
        
        if dist(Current_point, goal_point) <=5:
            r=500  #gain
            return r
        elif(collides(Current_point)== True):
            r=-1000
            return r
        else:
            r=0
            return 0
        
        
    def Update(q_Current_point, reward):
        
        global Q_table
        
        x = q_Current_point[0] // 5
        y = q_Current_point[1] // 5
        
        Q_table[y][x] = (1-gamma)*(Q_table[y][x])+gamma*(reward+loss*max_Q_nextvalue(q_Current_point))
        
    def Update_reward_Q_table(Current_point, reward):
        
        global Q_table
        x = Current_point[0] // 5
        y = Current_point[1] // 5
        Q_table[y][x] = reward
        
    
    def max_Q_nextvalue(Current_point):
        
        x = Current_point[0] // 5
        y = Current_point[1] // 5
        if ((x<=0)and (y<=0)):
            max_Q = np.max([Q_table[y][x+1],Q_table[y+1][x], Q_table[y+1][x+1]])
            return max_Q
        
        elif((x>= 145) and (y<=0)):
            max_Q = np.max([Q_table[y][x-1], Q_table[y+1][x], Q_table[y+1][x+1]])
            return max_Q
        
        elif((x<= 145) and (y>=90)):
            maxQ = np.max([Q_table[y-1][x], Q_table[y][x+1], Q_table[y][x+1]])
            return max_Q
        
        elif(x<= 0):
            max_Q = np.max([Q_table[y-1][x], Q_table[y][x+1], Q_table[y][x+1], Q_table[y+1][x+1], Q_table[y-1][x+1]])
            return max_Q
        
        elif(y<= 0):
            max_Q = np.max([Q_table[y][x-1], Q_table[y][x+1], Q_table[y+1][x], Q_table[y+1][x+1], Q_table[y+1][x-1]])
            return max_Q
        
        elif(x>= 145):
            max_Q = np.max([Q_table[y][x-1], Q_table[y-1][x], Q_table[y+1][x], Q_table[y+1][x-1], Q_table[y-1][x-1]])
            return max_Q
        
        elif(y>= 90):
            max_Q = np.max([Q_table[y][x-1], Q_table[y-1][x], Q_table[y][x+1], Q_table[y-1][x+1], Q_table[y-1][x-1]])
            return max_Q
        else:
            max_Q=np.max([Q_table[y][x-1],Q_table[y-1][x],Q_table[y][x+1],Q_table[y+1][x],Q_table[y+1][x+1],Q_table[y-1][x+1],Q_table[y+1][x-1],Q_table[y-1][x-1]])
            return max_Q   
        
        
    def max_Q_action(Current_point):
        
        x = Current_point[0] // 5
        y = Current_point[1] // 5
        
        if(x<=1):
            x=1
        if(y<=1):
            y=1
        if(x>=145):
            x=145
        if(y>=90):
            y=90
        greedy_action=np.argmax([Q_table[y][x-1],Q_table[y-1][x],Q_table[y][x+1],Q_table[y+1][x],Q_table[y+1][x+1],Q_table[y-1][x+1],Q_table[y+1][x-1],Q_table[y-1][x-1]])
        #print(greedy_action,Q_table[y][x-1],Q_table[y-1][x],Q_table[y][x+1],Q_table[y+1][x],Q_table[y+1][x+1],Q_table[y-1][x+1],Q_table[y+1][x-1],Q_table[y-1][x-1])
        if(greedy_action==0):
            return [5*(x-1),5*y]
        elif(greedy_action==1):
            return [5*x,5*(y-1)]
        elif(greedy_action==2):
            return [5*(x+1),5*y]
        elif(greedy_action==3):
            return [5*x,5*(y+1)]
        elif(greedy_action==4):
            return [5*(x+1),5*(y+1)]
        elif(greedy_action==5):
            return [5*(x+1),5*(y-1)]
        elif(greedy_action==6):
            return [5*(x-1),5*(y+1)]
        elif(greedy_action==7):
            return [5*(x-1),5*(y-1)]

        
    def random_action(Current_Point) :

        x=Current_Point[0]//5
        y=Current_Point[1]//5
        if((y<=0)and(x<=0)):
            return[Current_Point, [5*(x+1),5*(y+1)]]
        elif((x<=0)and(y>=99)):
            return [Current_Point,[5*(x+1),5*(y-1)]]
        elif((x>=160)and(y<=0)):
            return [Current_Point,[5*(x-1),5*(y+1)]]
        elif((x>=160)and(y>=99)):
            return [Current_Point,[5*(x-1),5*(y-1)]]
        elif(y<=0):
            random_value=np.random.randint(4,9)
            if(random_value==4):
                return [Current_Point,[5*(x-1),5*y]]
            elif(random_value==5):
                return [Current_Point,[5*(x+1),5*y]]
            elif( 6<=random_value<=8):
                return [Current_Point,[5*(x-7+random_value),5*(y+1)]]
        elif(y>=99):
            random_value=np.random.randint(1,6)
            if(1<= random_value<=3):
                return [Current_Point,[5*(x-2+random_value),5*(y-1)]]
            elif(random_value==4):
                return [Current_Point,[5*(x-1),5*y]]
            elif(random_value==5):
                return [Current_Point,[5*(x+1),5*y]]
        elif(x<=0):
            random_value=np.random.randint(4,9)
            if(random_value==4):
                return [Current_Point,[5*x,5*(y-1)]]
            elif(random_value==5):
                return [Current_Point,[5*x,5*(y+1)]]
            elif( 6<=random_value<=8):
                return [Current_Point,[5*(x+1),5*(x-7+random_value)]]
        elif(x>=160):
            random_value=np.random.randint(1,6)
            if(1<= random_value<=3):
                return [Current_Point,[5*(x-1),5*(y-2+random_value)]]
            elif(random_value==4):
                return [Current_Point,[5*x,5*(y-1)]]
            elif(random_value==5):
                return [Current_Point,[5*x,5*(y+1)] ]
        else:
            random_value=np.random.randint(1,9)
            if(1<= random_value<=3):
                return [Current_Point,[5*(x-2+random_value),5*(y-1)]]
            elif(random_value==4):
                return [Current_Point,[5*(x-1),5*y]]
            elif(random_value==5):
                return [Current_Point,[5*(x+1),5*y]]
            elif( 6<=random_value<=8):
                return [Current_Point,[5*(x-7+random_value),5*(y+1)]]


    def judge_goalfound(Current_Point,Goal_point):

        if(dist(Current_Point,Goal_point)<=6):
            return True
        return False

    
    
class Node:
    
    def __init__(self):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()    
        
        
class RRT(__Q_table):
    
    def __init__(self, start, goal, 
                 obstacleList,  incremental_dist = 15.0, 
                 learning_rate=20, max_iterations = 2000, randArea= None):
        
        self.start = Node(start[0], start[1])
        self.end   = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.margin = incremental_distance
        self.learning_rate = learning_rate
        self.obstacleList = obstacleList
        self.max_iterations = max_iterations
        
        
    def planning(self, animation = True):
        
        self.NodeList = {0 : self.start}
        i=0
        
        while True:
            print(i)
        if set(self.start).intersection(obstacleList) == None:
            self.NodeList.append(self.start)
            
            print(self.NodeList)
            
        rnd = self.get_random_point()
        nearest_index = self.GetNearestListIndex(rnd)
        new_node = self.steer(rnd, nearest_index)
        
        if self.Collision_check(new_node, self.obstacleList):
            
            near_indices = self.find_near_nodes(new_node, 5)
            new_node     = self.choose_parent(new_node, near_indices)
            self.nodeList[i+100] = new_node 
            self.rewire(i+100, new_node, near_indices)
            self.nodeList[new_node.parent].children.add(i+100)
            
            if len(self.NodeList) > self.max_iterations:
                leaves = [keys for key, node in self.NodeList.items]
                if len(leaves) > 1:
                    index = leaves[random.randint(0, len(leaves)-1)]
                    self.NodeList[self.NodeList[index].paresnt].children.discard(index)
                    self.NodeList.pop(index)
                else:
                    leaves = [key for key, node in self.NodeList.items() if len(node.children)==0]
                    index = leaves[random.randint(0, len(leaves) -1)]
                    self.NodeList[self.NodeList[index].parent].children.discard(index)
                    self.NodeList.pop(index)
                    
            if animation == True:
                self.DrawGraph(rnd)
                
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.obstacleList.append((event.pos[0], event.pos[1], 30, 30))
                        self.path_validation()
                    elif event.button == 3:
                        self.end.x = event.pos[0]
                        self.end.y = event.pos[1]
                        self.path_validation()
                        
                        
    def path_validation(self):
        
        lastIndex = self.get_the_best_last_index()
        if lastIndex and set(lastIndex).intersection(obstacleList) == None:
            while self.NodeList[lastIndex].parent is not None:
                nodeIndex = lastIndex
                lastIndex = self.NodeList[lastIndex].parent
                
                dx = self.NodeList[nodeIndex].x - self.NodeList[lastIndex].x
                dy = self.NodeList[nodeIndex].y - self.NodeList[lastIndex].y
                d = math.sqrt.atan2(dx**2 + dy**2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.NodeList[lastIndex].x,self.NodeList[lastIndex].y, theta, d):
                    self.NodeList[lastIndex].children.discard(nodeIndex)
                    self.eliminate_branch(nodeIndex)

    def eliminate_brach(self, nodeIndex):
        safenodesList = []
        if set(nodeIndex).intersection(obstacleList) == None:
            safenodesList.append(nodeIndex)
        for not_safe in safenodesList[nodeIndex].children:
            self.eliminate_branch(not_safe)
        self.NodeList.pop(nodeIndex)
     
    
    def choose_parent(self,new_node,nearest_index):
        if len(nearest_index) == 0:
            return new_node
        
        if Current_point == nearest_index and self.CollisionCheck(new_node, obstacleList):
            
            Current_point = new_node
            return Current_point
        
        distanceList = []
        for i in near_indices:
            dx = new_node.x - self.NodeList[i].x
            dy = new_node.y - self.NodeList[i].y
            d = math.sqrt(dx **2 + dy **2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.NodeList[i].x, self.NodeList[i].y, theta, d):
                distanceList.append(self.NodeList[i].cost + d)
            else:
                distanceList.append(float("inf"))
                
        minimum_cost = min(distanceList)
        minimum_index = near_indices[distacelist.index(minimum_cost)]
        
        if minimum_cost == float("inf"):
            print("minimum_cost is INFINITE")
            return Current_point
        
        Current_point.cost = minimum_cost
        Current_point.parent = minimum_index
         
        Current_point_with_maximum_Q_value = self.max_Q_nextvalue(Current_point)
        
        return Current_point.cost, Current_point.parent, Current_point_with_maximum_Q_value
    
    
        
    def steer(self, rnd, nearest_index):
        
        nearest_node = self.NodeList[nearest_index]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(nearest_node.x , nearest_node.y)
        new_node.x += incremental_distance * math.cos(theta)
        new_node.y += incremental_distance * math.sin(theta)
        
        new_node.cost = nearest_node.cost + incremental_distance
        new_node.parent = nearest_index 
        return new_node
    
            
    def get_random_point(self, Current_point=None):
        
        self.Current_point = Current_point
        if random.randint(0, 100) > self.learning_rate:
            rnd = [random.uniform(0, Xrand), random.uniform(0, yrand)]
            
        elif random.randint(0, 100) <= RANDOM_COUNT:
            Current_point = max_Q_action(Current_point)
            rnd = random_action(Current_point)
        else:
            rnd = [self.end.x, self.end.y]
        return rnd
    
    def get_best_last_index(self):

        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.NodeList.items()]
        goal_indices = [key for key, distance in disglist if distance <= self.margin]

        if len(goal_indices) == 0:
            return None

        minimum_cost = min([self.NodeList[key].cost for key in goal_indices])
        for i in goal_indices:
            if self.NodeList[i].cost == minimum_cost:
                return i

        return None

    def gen_final_course(self, goal_indices):
        path = [[self.end.x, self.end.y]]
        while self.NodeList[goal_indices].parent is not None:
            node = self.NodeList[goal_indices]
            path.append([node.x, node.y])
            goal_indices = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node, value):
        r = self.margin * value

        distanceList = np.subtract( np.array([ (node.x, node.y) for node in self.NodeList.values() ]), (new_node.x,new_node.y))**2
        distanceList = np.sum(distanceList, axis=1)
        near_indices = np.where(distanceList <= r ** 2)
        near_indices = np.array(list(self.NodeList.keys()))[near_indices]

        return nearinds

    def rewire(self, newNodeInd, new_node, near):
        nnode = len(self.NodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = new_node.x - nearNode.x
            dy = new_node.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if near_node.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode.x, nearNode.y, theta, d):
                    self.NodeList[nearNode.parent].children.discard(i)
                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    new_node.children.add(i)

    def check_collision_extend(self, nix, niy, theta, d):

        tmpNode = Node(nix,niy)

        for i in range(int(d/5)):
            tmpNode.x += 5 * math.cos(theta)
            tmpNode.y += 5 * math.sin(theta)
            if not self.CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):

        screen.fill((255, 255, 255))
        for node in self.NodeList.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.NodeList[node.parent].x,self.NodeList[node.parent].y],[node.x,node.y])

        for node in self.NodeList.values():
            if len(node.children) == 0: 
                pygame.draw.circle(screen, (255,0,255), [int(node.x),int(node.y)], 2)
                

        for(sx,sy,ex,ey) in self.obstacleList:
            pygame.draw.rect(screen,(0,0,0), [(sx,sy),(ex,ey)])

        pygame.draw.circle(screen, (255,0,0), [self.start.x, self.start.y], 10)
        pygame.draw.circle(screen, (0,0,255), [self.end.x, self.end.y], 10)

        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            path = self.gen_final_course(lastIndex)

            ind = len(path)
            while ind > 1:
                pygame.draw.line(screen,(255,0,0),path[ind-2],path[ind-1])
                ind-=1

        pygame.display.update()
    
    
    def Get_nearest_list_index(self, rnd):
        distanceList = np.subtract( np.array([ (node.x, node.y) for node in self.NodeList.values() ]), (rnd[0],rnd[1]))**2
        distanceList = np.sum(distanceList, axis=1)
        minimum_index = list(self.NodeList.keys())[np.argmin(distancelist)]
        return minimum_index

    def Collision_check(self, node, obstacleList):
        for(sx,sy,ex,ey) in obstacleList:
            sx,sy,ex,ey = sx+2,sy+2,ex+2,ey+2
            if node.x > sx and node.x < sx+ex:
                if node.y > sy and node.y < sy+ey:
                    return False

        return True  
    
    
def main():
    print("start RRT path planning")


    obstacleList = [
        (400, 380, 400, 20),
        (400, 220, 20, 180),
        (500, 280, 150, 20),
        (0, 500, 100, 20),
        (500, 450, 20, 150),
        (400, 100, 20, 80),
        (100, 100, 100, 20)
    ]  

    rrt = RRT(obstacleList = obstacleList, start =[10, 580], goal = [540, 150],
              randArea = [XDIM, YDIM])
    
    path = rrt.planning(animation=show_animation)


if __name__ == '__main__':
    main()  
    


# In[ ]:




