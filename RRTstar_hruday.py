import math
import random
import matplotlib.pyplot as plt

# size = int(input("Enter the size of graph : "))
# step = int(input("Enter the size of max step :"))
# nstep = int(input("Enter the size of the neighbourhood :"))
# maxiter = int(input("Enter the max iter :"))
size = 1000
step = 50
nstep = 80
maxiter = 500
obstaclelist = [(350,650,50),(100,300,70),(500,250,90),(750,750,75)]

class treenode():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

    def setcost(self,cost):
        self.cost = cost

    def getcost(self):
        return self.cost

    def setparent(self,node):
        self.parent = node

    def getparent(self):
        return self.parent

    def printposition(self):
        print(str(self.x)+" "+ str(self.y))

def drawplot(nodes,goal):
    plt.figure(figsize=(7, 7))
    plt.grid(False)
    plt.axis([0, size, 0, size])

    for obstacle in obstaclelist:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2],color = 'grey', fill=True)
        plt.gca().add_artist(circle)

    for node in nodes:
        if(node.parent):
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')

    node = goal
    while(node.parent):
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-r')
        node = node.parent

    plt.show()

def updatecost(nodes):
    for node in nodes:
        if(node.parent):
            cost = (distance(node,node.parent))+node.parent.getcost()
            node.setcost(cost)

def distance(node1,node2):
    return math.hypot(node1.x - node2.x, node1.y - node2.y)

def getrandomnode():
    return treenode(random.randint(0,size),random.randint(0,size))

def nearestnode(nodes,newnode):
    nearnode = nodes[0]
    mindist= distance(nearnode,newnode)

    for node in nodes:
        dist = distance(node,newnode)
        if(mindist>dist):
            nearnode = node
            mindist = dist
    
    return nearnode

def isneargoal(goal,newnode,step):
    if(distance(goal,newnode)<step):
        return True

    return False

def neighbourhood(newnode,nstep,nodes):
    nbhnodes = []
    for node in nodes:
        if(distance(node,newnode)<nstep):
            nbhnodes.append(node)
    return nbhnodes

def printway(goal):
    node = goal
    node.printposition()
    while(node.parent):
        node = node.getparent()
        node.printposition()
        print(node.getcost())

def shortstep(newnode,nearnode,step):
    if(distance(newnode,nearnode)>step):
        x = nearnode.x+ ((newnode.x - nearnode.x)/distance(newnode,nearnode))*step
        y = nearnode.y+ ((newnode.y - nearnode.y)/distance(newnode,nearnode))*step
        return treenode(x,y)
    return newnode

def bestparent(newnode,nbhnodes,nearnode):
    mincost = distance(newnode,nearnode) + nearnode.getcost()
    parent = nearnode
    for node in nbhnodes:
        newcost = distance(newnode,node) + node.getcost()
        if(newcost<mincost):
            parent = node
            mincost = newcost
    return parent

def rewire(newnode,nbhnodes):
    for node in nbhnodes:
        if(node.getcost()>(newnode.getcost()+distance(newnode,node))):
            node.setparent(newnode)
            node.setcost(newnode.getcost()+distance(newnode,node))

def intersectingobstacle(node1, node2):
    for obstacle in obstaclelist:
        x, y, radius = obstacle
        if intersectscircle(node1.x, node1.y, node2.x, node2.y, x, y, radius):
            return True
    return False

def intersectscircle(x1, y1, x2, y2, x, y, radius):
    px, py = x2 - x1, y2 - y1
    norm = px*px + py*py
    u = ((x - x1) * px + (y - y1) * py) / float(norm)
    u = max(min(u, 1), 0)  
    closestx = x1 + u * px
    closesty = y1 + u * py
    dist_to_line = math.hypot(closestx - x, closesty - y)
    return dist_to_line < radius

def rrtstar(start,goal):
    nodes = [start]
    pathfound = False

    while(len(nodes)!=maxiter):

        newnode = getrandomnode()
        nearnode = nearestnode(nodes,newnode)
        newnode = shortstep(newnode,nearnode,step)

        if intersectingobstacle(nearnode, newnode):
            continue 
        
        nbhnodes  = neighbourhood(newnode,nstep,nodes)
        nearnode = bestparent(newnode,nbhnodes,nearnode)
        nbhnodes.remove(nearnode)

        newnode.setparent(nearnode)
        newnode.setcost(distance(newnode,nearnode)+nearnode.getcost())
        nodes.append(newnode)
        rewire(newnode,nbhnodes)
        updatecost(nodes)

        if(isneargoal(goal,newnode,step)):
            goal.setparent(newnode)
            pathfound = True
            
    if(pathfound): drawplot(nodes,goal)
    printway(goal)

start = treenode(50,50)
goal = treenode(500,500)

rrtstar(start,goal)