from map import Map
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np
import math
import random
import matplotlib.patches as patches
from utils import Utils
from node import Node
import ros_messenger
import time

class IRrtStar:
    def __init__(self, start_position, goal_position, step_len, sample_rate, search_radius, iter_max):
        self.start_position = Node(start_position)
        self.goal_position = Node(goal_position)
        self.step_len = step_len
        self.sample_rate = sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.utils = Utils()
        self.map = Map()
        self.clearance = self.utils.clearance

        self.fig, self.ax = plt.subplots()
        self.x_range = self.map.x_range
        self.y_range = self.map.y_range

        self.obs_circle = self.map.obs_circle
        self.obs_rectangle = self.map.obs_rectangle

        self.V = [self.start_position]
        self.X_soln = set()
        self.path = None
    
    def init(self):
        cMin, theta = self.get_dist_and_ang(self.start_position, self.goal_position)
        C = self.R2W(self.start_position, self.goal_position, cMin)
        center_x = np.array([[(self.start_position.x + self.goal_position.x) / 2.0],
                            [(self.start_position.y + self.goal_position.y) / 2.0], [0.0]])
        best_x = self.start_position
        return theta, cMin, center_x, C, best_x

    def main_algo(self):
        theta, cMin, center_x, C, best_x = self.init()
        c_best = np.inf

        for iter in range(self.iter_max):
            if self.X_soln:
                cost = {node: self.Cost(node) for node in self.X_soln}
                best_x = min(cost, key=cost.get)
                c_best = cost[best_x]
            rand_x = self.Sample(c_best, cMin, center_x, C)
            x_nearest = self.Nearest(self.V, rand_x)
            x_new = self.Steer(x_nearest, rand_x)
            if x_new and not self.utils.isCollision(x_nearest, x_new):
                X_near = self.Near(self.V, x_new)
                c_min = self.Cost(x_nearest) + self.Line(x_nearest, x_new)
                self.V.append(x_new)

                # choose parent
                for x_near in X_near:
                    c_new = self.Cost(x_near) + self.Line(x_near, x_new)
                    if c_new < c_min:
                        x_new.parent = x_near
                        c_min = c_new
                
                for x_near in X_near:
                    c_near = self.Cost(x_near)
                    c_new = self.Cost(x_new) + self.Line(x_new, x_near)
                    if c_new < c_near:
                        x_near.parent = x_new

                if self.InGoalRegion(x_new):
                    if not self.utils.isCollision(x_new, self.goal_position):
                        self.X_soln.add(x_new)
            
            if iter % 20 == 0:
                self.animation(x_center=center_x, c_best=c_best,
                               dist=cMin, theta=theta)
        self.path = self.ExtractPath(best_x)
        self.animation(x_center=center_x, c_best=c_best,
                       dist=cMin, theta=theta)
        
        plt.plot([x for x, _ in self.path], [y for _, y in self.path], '-r')
        plt.pause(0.01)
        plt.show()
        # anim = animation.FuncAnimation(self.fig, self.animation, frames=1000, 
        #                        interval=20, blit=True)
        # # fig.suptitle('Straight Line plot', fontsize=14)
        
        # # saving to m4 using ffmpeg writer
        # writervideo = animation.FFMpegWriter(fps=60)
        # anim.save('RRT_star.mp4', writer=writervideo)
        return self.path[::-1]

    def Cost(self, node):
        if node == self.start_position:
            return 0.0

        if node.parent is None:
            return np.inf

        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost
    
    def InGoalRegion(self, node):
        if self.Line(node, self.goal_position) < self.step_len:
            return True

        return False
    
    def Near(self, nodelist, node):
        n = len(nodelist) + 1
        r = 50 * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y)
                      ** 2 for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if
                  dist_table[ind] <= r ** 2
                  and not self.utils.isCollision(nodelist[ind], node)]

        return X_near
    
    def Steer(self, x_start, x_goal):
        dist, theta = self.get_dist_and_ang(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new
    
    @staticmethod
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]
    
    @staticmethod
    def Line(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)
    
    def Sample(self, c_max, c_min, x_center, C):
        if c_max < np.inf:
            d = c_max ** 2 - c_min ** 2
            if d < 0:
                d = 0.01
            r = [c_max / 2.0,
                 math.sqrt(d) / 2.0,
                 math.sqrt(d) / 2.0]
            L = np.diag(r)

            while True:
                x_ball = self.SampleUnitBall()
                x_rand = np.dot(np.dot(C, L), x_ball) + x_center
                if self.x_range[0] + self.clearance <= x_rand[0] <= \
                        self.x_range[1] - self.clearance and self.y_range[0] + \
                        self.clearance <= x_rand[1] <= self.y_range[1]-self.clearance:
                    break
            x_rand = Node((x_rand[(0, 0)], x_rand[(1, 0)]))
        else:
            x_rand = self.SampleFreeSpace()

        return x_rand
    
    def ExtractPath(self, node):
        path = [[self.goal_position.x, self.goal_position.y]]

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.start_position.x, self.start_position.y])

        return path
    
    @staticmethod
    def SampleUnitBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    @staticmethod
    def get_dist_and_ang(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    
    @staticmethod
    def R2W(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U)
                        * np.linalg.det(V_T.T)]) @ V_T

        return C
    
    def SampleFreeSpace(self):
        delta = self.clearance

        if np.random.random() > self.sample_rate:
            return Node((np.random.uniform
                         (self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta,
                                           self.y_range[1] - delta)))

        return self.goal_position
    
    def animation(self, x_center=None, c_best=None, dist=None, theta=None):
        plt.cla()
        self.plot_grid("Informed rrt*, N = " + str(self.iter_max))
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        for node in self.V:
            if node.parent:
                plt.plot([node.x, node.parent.x], [
                            node.y, node.parent.y], "-b")

        if c_best != np.inf:
            self.draw_ellipse(x_center, c_best, dist, theta)

        plt.pause(0.01)

    def plot_grid(self, name):

        for (ox, oy, w, h) in self.utils.vertices:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        # for (ox, oy, r) in self.obs_circle:
        ox, oy, r = self.obs_circle
        self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.start_position.x, self.start_position.y, "bs", linewidth=3)
        plt.plot(self.goal_position.x, self.goal_position.y, "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")
    
    def draw_ellipse(self, x_center, c_best, dist, theta):
        temp = c_best ** 2 - dist ** 2
        if temp < 0:
            temp = 0.01
        a = math.sqrt(temp) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, ".b")
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)



def main():
    x_start = (5,10)  # Starting point
    x_goal = (32,15)  # Goal point

    # start_position, goal_position, step_len, sample_rate, search_radius, iter_max
    rrt_star = IRrtStar(x_start, x_goal, 1, 0.10, 10, 2000)
    a = time.time()
    path = rrt_star.main_algo()
    b = time.time()
    print("Time Taken:", b-a)
    print('Path Found:')
    print(path)
    value = int(input("Do you want the turtlebot to move? type 1 to move it: \n"))
    if value == 1:
        ros_messenger.send_vel(path, radius=3.8, w_dist=35.4, step=1)

if __name__ == '__main__':
    main()