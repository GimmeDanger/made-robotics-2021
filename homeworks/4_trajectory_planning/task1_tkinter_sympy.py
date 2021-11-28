from tkinter import *
from heapq import *
import math
import numpy as np
from sympy import Point, Polygon
from copy import deepcopy
from collections import deque

'''================= Your classes and methods ================='''

# These functions will help you to check collisions with obstacles

def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new+cx, y_new+cy))

    return new_points

def get_polygon_from_position(position) :
    x,y,yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)] 
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    return Polygon(*list(map(Point, new_points)))

def get_polygon_from_obstacle(obstacle) :
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])] 
    return Polygon(*list(map(Point, points)))

def get_polygon_from_points(points) :
    return Polygon(*list(map(Point, points)))

def poly_collides(poly_lhs, poly_rhs) :
    return poly_lhs.intersection(poly_rhs) or \
        True in [poly_lhs.encloses_point(p) for p in poly_rhs.vertices]

def collides(position, obstacle) :
    return poly_collides(get_polygon_from_position(position),
                         get_polygon_from_obstacle(obstacle))


class State():

    def __init__(self, position, hash_pos=False):

        self.coord_prec = 1
        self.angle_prec = 0.001
        self.length = 150

        x, y, yaw = position
        if hash_pos:
            x = self.discretize(x, self.coord_prec, reverse=True)
            y = self.discretize(y, self.coord_prec, reverse=True)
            yaw = self.discretize(yaw, self.angle_prec, reverse=True)
        yaw = yaw + (1.5 * math.pi)
        yaw %= (2 * math.pi)

        self.x = x
        self.y = y
        self.yaw = yaw

    def to_hash(self, do_discretize=True):
        x = self.x
        y = self.y
        yaw = (self.yaw - 1.5 * math.pi + 2 * math.pi) % (2 * math.pi)
        if do_discretize:
            return self.discretize(x, self.coord_prec), \
                   self.discretize(y, self.coord_prec), \
                   self.discretize(yaw, self.angle_prec)
        else:
            return x, y, yaw

    def discretize(self, x, prec, reverse=False):
        if not reverse:
            return int(x / prec)
        else:
            return x * prec

    def move(self, control):
        steer, d = control
        if abs(steer) < 1e-16:
            self.x = self.x + d * math.cos(self.yaw)
            self.y = self.y + d * math.sin(self.yaw)
        else:
            R = self.length / math.tan(steer)
            beta = d / R
            x_c = self.x - R * math.sin(self.yaw)
            y_c = self.y + R * math.cos(self.yaw)
            self.x = x_c + R * math.sin(self.yaw + beta)
            self.y = y_c - R * math.cos(self.yaw + beta)
            self.yaw = (self.yaw + beta) % (2 * math.pi)


class Window():
        
    '''================= Your Main Function ================='''


    def get_line_path(self, eps=1.e-8, delta=1.e-2, max_iters=1.e+8):
        points = []
        s_x, s_y, _ = self.get_start_position()
        t_x, t_y, _ = self.get_target_position()
        # ax + by + k = 0
        if abs(s_x - t_x) < eps and abs(s_y - t_y) < eps:
            return points
        elif abs(s_x - t_x) < eps:
            a = 0
            b = 1
            k = -s_y
        elif abs(s_y - t_y) < eps:
            a = 1
            b = 0
            k = -s_x
        else:
            a = -1 * (s_y - t_y) / (s_x - t_x)
            b = 1
            k = -1 * (a * s_x + b * s_y)
        iter = 0
        sgn_x = 1 if t_x > s_x else -1
        sgn_y = 1 if t_y > s_y else -1
        while abs(s_x - t_x) > delta or abs(s_y - t_y) > delta:
            points.append((s_x, s_y))
            if abs(a) < eps:
                s_y += delta * sgn_y
            elif abs(b) < eps:
                s_x += delta * sgn_x
            else:
                s_x += delta * sgn_x
                s_y = (-k - a * s_x) / b
            if iter > max_iters:
                print("Error: cannot construct path")
                return []
            iter += 1

        return points
    

    def get_motion_path(self, steer=0.0, d=20, steps=20):
        points = []
        st = State(self.get_start_position())
        for i in range(0, steps):
            st.move(control=(steer, d))
            print(st.x, st.y, st.yaw)
            points.append((st.x, st.y))
        return points


    def get_error(self, h_lhs, h_rhs):
        lhs = State(h_lhs, hash_pos=True)
        rhs = State(h_rhs, hash_pos=True)
        x_lhs, y_lhs, yaw_lhs = lhs.x, lhs.y, lhs.yaw
        x_rhs, y_rhs, yaw_rhs = rhs.x, rhs.y, rhs.yaw
        x = abs(x_lhs - x_rhs)
        y = abs(y_lhs - y_rhs)
        yaw = 500 * abs(yaw_lhs - yaw_rhs)
        return math.sqrt(x * x + y * y + yaw * yaw)


    def a_star_h(self, h_lhs, h_rhs):
        return self.get_error(h_lhs, h_rhs)


    def get_path_a_star(self):
        source = State(self.get_start_position()).to_hash()
        target = State(self.get_target_position()).to_hash()
        visited = set()
        prev = dict()
        g = dict()
        g[source] = 0
        f = g[source] + self.a_star_h(source, target)
        pq = []
        heappush(pq, (f, source))

        closest, smallest_err = None, 1e+32
        max_iters = 2000000
        iter = 0

        while pq:
            _, current = heappop(pq)
            err = self.get_error(current, target)
            if err < smallest_err:
                smallest_err = err
                closest = current
            # print(iter, smallest_err, clossest)
            iter += 1
            if iter % (max_iters / 100) == 0:
                print(f'iter = {iter}, smallest_err = {smallest_err}')
            
            if current == target or iter == max_iters:
                if iter == max_iters:
                    cls = State(closest, hash_pos=True)
                    print(f'max iters reached: closest = {cls.to_hash(False)}, err = {smallest_err}')
                else:
                    print(f'target reached')
                break
            if current in visited:
                continue
            visited.add(current)

            d = self.a_star_h(current, target) / 10
            for steer in [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3]:
                v = State(current, hash_pos=True)
                v.move(control=(steer, d))
                v = v.to_hash()
                score = g[current] + self.a_star_h(current, v)
                if v in visited and score >= g[v]:
                    continue
                if v not in visited or score < g[v]:
                    prev[v] = current
                    g[v] = score
                    f = score + self.a_star_h(v, target)
                    heappush(pq, (f, v))
        # reconstruct
        rpath = []
        v = closest
        while v != source:
            st = State(v, hash_pos=True)
            rpath.append((st.x, st.y))
            v = prev[v]
        st = State(v, hash_pos=True)
        rpath.append((st.x, st.y))
        print(f'total path length = {len(rpath)}')
        return rpath


    def get_path_bfs(self):
        source = State(self.get_start_position()).to_hash()
        target = State(self.get_target_position()).to_hash()
        visited = set()
        prev = dict()
        q = deque()
        q.append(source)
        while len(q) > 0:
            h = q.popleft()
            if h == target:
                print("path found")
                break
            if h in visited:
                continue
            visited.add(h)
            for steer in [-0.2, -0.1, 0.0, 0.1, 0.2]:
                adj = State(h, hash_pos=True)
                adj.move(control=(steer, 10))
                adj = adj.to_hash()
                if adj not in visited:
                    q.append(adj)
                    prev[adj] = h
        # reconstruct
        rpath = []
        v = target
        while v != source:
            st = State(v, hash_pos=True)
            rpath.append((st.x, st.y))
            v = prev[v]
        st = State(v, hash_pos=True)
        rpath.append((st.x, st.y))
        return rpath


    
    def go(self, event):
    
        # Write your code here
                
        print("Start position:", self.get_start_position())
        print("Target position:", self.get_target_position()) 
        print("Obstacles:", self.get_obstacles())
        
        # Example of collision calculation
        start_poly = get_polygon_from_position(self.get_start_position())
        target_poly = get_polygon_from_position(self.get_target_position())
        path = self.get_path_a_star()
        max_printable = 20
        step = int(len(path) / max_printable) if len(path) > max_printable else 1
        for x, y in path[::step]:
            points = [(x - 5, y - 5), (x + 5, y - 5), (x + 5, y + 5), (x - 5, y + 5)] 
            # poly = get_polygon_from_points(points)
            # if poly_collides(start_poly, poly):
            #     continue
            # if poly_collides(target_poly, poly):
            #     continue
            self.path_object_ids.add(self.draw_block(points, "#ff0000"))
        
        number_of_collisions = 0
        for obstacle in self.get_obstacles() :
            if collides(self.get_start_position(), obstacle) :
                number_of_collisions += 1
        print("Start position collides with", number_of_collisions, "obstacles")


    def delete_path(self):
        for id in self.path_object_ids:
            self.canvas.delete(id)
        self.path_object_ids = set()
        
        
    '''================= Interface Methods ================='''
    
    def get_obstacles(self) :
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if i > 2 and i not in self.path_object_ids:
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles
            
            
    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        yaw = self.get_yaw(2)
        return x,y,yaw
    
    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1 
        yaw = self.get_yaw(1)
        return x,y,yaw 
 

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y 
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw)
        else :
            return -math.acos(cos_yaw)
       
    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):

        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        self.delete_path()
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        self.delete_path()
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        self.delete_path()

        block = [[0, 100], [100, 100], [100, 300], [0, 300]]
        id = self.draw_block(block, "black")
        print(id)

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width  = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        self.path_object_ids = set()
    
if __name__ == "__main__":
    run = Window()
    run.run()
