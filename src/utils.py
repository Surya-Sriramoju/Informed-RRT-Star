from node import Node
import numpy as np
import math
from map import Map

class Utils:
    def __init__(self):
        self.clearance = 1.5
        self.map = Map()
        self.circle = self.map.obs_circle
        self.rectangles = self.map.obs_rectangle
        self.vertices = self.map.obs_boundary
    
    def rec_vertex(self):
        vertices = []

        for (x,y,w,h) in self.rectangles:
            vertices_list = [[x-self.clearance, y-self.clearance],
                             [x+w+self.clearance, y-self.clearance],
                             [x+w+self.clearance, y+h+self.clearance],
                             [x-self.clearance, y+h+self.clearance]]
            vertices.append(vertices_list)
        # print(len(vertices))
        return vertices
    
    def rect_intersect(self, start, end, origin, direction, vert1, vert2):
        v1 = [origin[0] - vert1[0], origin[1] - vert1[1]]
        v2 = [vert2[0] - vert1[0], vert2[1] - vert1[1]]
        v3 = [-direction[1], direction[0]]

        

        division = np.dot(v2, v3)
        if division == 0:
            return False
        
        t1 = np.linalg.norm(np.cross(v2, v1))/division
        t2 = np.dot(v1, v3) / division

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((origin[0] + t1 * direction[0], origin[1]+ t1 * direction[1]))
            dist_obj = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obj <= dist_seg:
                return True
        
        return False
    
    def circle_intersect(self, origin, direction, coord, r):
        division = np.dot(direction, direction)
        if division == 0:
            return False
        t = np.dot([coord[0] - origin[0], coord[1] - origin[1]], direction) / division

        if 0 <= t <= 1:
            shot = Node((origin[0] + t * direction[0], origin[1]+ t * direction[1]))
            if self.get_dist(shot, Node(origin)) <= r + self.clearance:
                return True
        return False


    def isCollision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True
        # print(start.x,start.y)
        # print(end.x, end.y)
        origin, direction = self.get_ray(start, end)
        rect_vertices = self.rec_vertex()
        # print(rect_vertices)

        for (v1, v2, v3, v4) in rect_vertices:

            if self.rect_intersect(start, end, origin, direction, v1, v2):
                return True
            if self.rect_intersect(start, end, origin, direction, v2, v3):
                return True
            if self.rect_intersect(start, end, origin, direction, v3, v4):
                return True
            if self.rect_intersect(start, end, origin, direction, v4, v1):
                return True

        # for (x, y, r) in self.obs_circle:
        x,y,r = self.circle
        if self.circle_intersect(origin, direction, [x, y], r):
            return True

        return False
   
    def is_inside_obs(self, node):

        # for (x, y, r) in self.circle:
        x,y,r = self.circle
        if math.hypot(node.x - x, node.y - y) <= r + self.clearance:
            return True

        for (x, y, w, h) in self.rectangles:
            if 0 <= node.x - (x - self.clearance) <= w + 2 * self.clearance \
                    and 0 <= node.y - (y - self.clearance) <= h + 2 * self.clearance:
                return True

        for (x, y, w, h) in self.vertices:
            if 0 <= node.x - (x - self.clearance) <= w + 2 * self.clearance \
                    and 0 <= node.y - (y - self.clearance) <= h + 2 * self.clearance:
                return True
        # x1,y1,x2,y2 = self.vertices[0]

        # if(((x1 + self.clearance <= node.x <= x2 - self.clearance) and (y1 + self.clearance <= node.y <= y2 - self.clearance)) == False):
        #     return True
        # return False
    

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)