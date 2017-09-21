
import math
import pygame
import sys
import random

settings = { 'max_rotational_speed': math.radians(5),
             'delta_rotational_speed': math.radians(0.05),
             'thrust_accelleration': 0.05,
             'screen_size': [ 1000, 700]
           }

def f_eq(v1, v2, accuracy=0.001):
    return abs(v1 - v2) < accuracy

# Class to define a polar set of points (2-D)
class PolarPoints:

    def __init__(self):
        self.points = [ ]

        self.cached_points = { }

    @staticmethod
    def add_polar_points(a, b):
        v1 = [ a[0] * math.cos(a[1]),
               a[0] * math.sin(a[1])]
        v2 = [ b[0] * math.cos(b[1]),
               b[0] * math.sin(b[1])]
        
        # resultant vector 
        r = [ v1[0] + v2[0], v1[1] + v2[1] ]

        # convert to polar
        d = math.sqrt(r[0] * r[0] + r[1] * r[1])
        ang = 0
        if (r[1] >= 0):
            ang = math.acos(r[0] / d) if d > 0 else 0
        else:
            ang = (2*math.pi -math.acos(r[0] / d)) if d > 0 else 0

        return [d, ang] 

    @staticmethod
    def find_line_equation(a, b):
        """ from points a and b, find the equation y=kx+m.
            For a non vertical line, returns:
               [ k, m, xmin, xmax ]. 
            For a vertical line, return:
               [ None, x, ymin, ymax] 
        """ 
        if (b[0] != a[0]): 
            # Non vertical line
            x_min = a[0] if a[0] < b[0] else b[0]
            x_max = a[0] if a[0] > b[0] else b[0]
            k = (b[1] - a[1]) / (b[0] - a[0])
            m = a[1] - k*a[0] 
            return [ k, m, x_min, x_max ]
        else:
            # Vertical line
            y_min = a[1] if a[1] < b[1] else b[1]
            y_max = a[1] if a[1] > b[1] else b[1]
            return [ None, a[0], y_min, y_max ]

    def clear(self):
        self.points = [ ]

    # Add polar point to list of points
    def add_polar(self, d, a):
        self.points.append([d, a])

    # Add cartesian point to list of points
    def add_cartesian(self, point):
        x = point[0]
        y = point[1]

        d = math.sqrt(x * x + y * y)
        a = 0
        if (y >= 0):
            a = math.acos(x / d) if d > 0 else 0
        else:
            a = (2*math.pi -math.acos(x / d)) if d > 0 else 0

        x2 = d * math.cos(a)
        y2 = d * math.sin(a)
            
        self.add_polar(d, a)


    # Given a list of pairs of points, [ [a,b], [c,d], ... ], generate a set of
    # polar points and store them in the object
    def define_cartesian_points(self, points):
        self.points = [ ]
        for p in points:
            self.add_cartesian(p)
            

    def print(self):
        print("PolarPoints:")
        for p in self.points: 
            print("   p.d = %g,   p.a = %g rad (%g deg)" % (p[0], p[1], math.degrees(p[1])))

    
    def get_cartesian_points(self, rotation=0):
        # Check if we have this polygon in the cache already (max 1000 angles saved)
        if round(rotation * 1000) in self.cached_points: 
            return self.cached_points[round(rotation * 1000)]
        else:
            # Not in cache - calculate points
            cartesian = []
            for p in self.points:
                d = p[0]
                a = p[1]

                x = d * math.cos(a - rotation)
                y = d * math.sin(a - rotation)

                cartesian.append([round(x), round(y)])

            self.cached_points[round(rotation * 1000)] = cartesian
            return cartesian


class SimplePoly:
    def __init__(self, color=[255,255,255], fill=False, linewidth=1, scale=1.0):
    
        # pygame idiosyncracy: If linewidth for a polygon is 0, it is filled
        if fill:
            linewidth = 0

        self.linewidth = linewidth 
        self.color = color
        self.fill = fill
        self.points = PolarPoints()
        self.scale = scale

    def add_polarPoints(self, points):
        for p in points:
            self.points.add_polar(p)

    def add_cartesianPoints(self, points):
        self.points.define_cartesian_points(points)

    def get_cartesian_points(self, rotation=0):
        return self.points.get_cartesian_points(rotation)

    def clear_points(self):
        self.points.clear()

    def draw(self, screen, rotation, translation):
        cp = self.get_cartesian_points(rotation)
        
        actual_points = []
        for i in cp:
            x = i[0] if f_eq(self.scale, 1.0) else i[0] * self.scale
            y = i[1] if f_eq(self.scale, 1.0) else i[1] * self.scale

            x += translation[0]
            y += translation[1]

            actual_points.append([x, y])

        pygame.draw.polygon(screen, self.color, actual_points, self.linewidth)
        
    @staticmethod
    def box_overlap(a, b):
        """ Checks if two boxes a + b overlap """

        a_x1 = a[0]
        a_y1 = a[1]
        a_x2 = a[2]
        a_y2 = a[3]

        b_x1 = b[0]
        b_y1 = b[1]
        b_x2 = b[2]
        b_y2 = b[3]

        # Possible overlapping boxes (one axis)
        #       bx1  ax1   bx2  ax2
        #       ax1  bx1   ax2  bx2

        if (((b_x1 < a_x1) and (a_x1 < b_x2)) or 
            ((a_x1 < b_x1) and (b_x1 < a_x2))):

            if (((b_y1 < a_y1) and (a_y1 < b_y2)) or 
                ((a_y1 < b_y1) and (b_y1 < a_y2))):

                return True
        return False

    @staticmethod
    def line_overlap(a, b):
        a_k, a_m, a_min, a_max = a
        b_k, b_m, b_min, b_max = b

        if a_k is None:
            # line a is vertical
            # Note - for a vertical line, a_m and b_m are the x 
            # coordinates of the lines
            if b_k is None:
                # line b is also vertical
                if not f_eq(a_m, b_m):
                    return False
                else:
                    if a_max < b_min or a_min > b_max:
                        return False
                    if b_max < a_min or b_min > a_max:
                        return False
                    else:
                        return True

            else:
                # See if b can cross a:
                if b_min < a_m < b_max: 
                    # possibly - calculate y for b at a_m
                    y_b = b_k * a_m + b_m  # IS THIS CORRECT?
                    if a_min < y_b < a_max:
                        # crosses
                        return True
                    else:
                        return False
                else:
                    return False
        elif b_k is None:
            # See if a can cross b:
            if a_min < b_m < a_max: 
                # possibly - calculate y for a at b_m
                y_a = a_k * b_m + a_m  # IS THIS CORRECT?
                if b_min < y_a < b_max:
                    # crosses
                    return True
                else:
                    return False
            else:
                return False

        else:
            # straight forward cross check
            if a_max < b_min or b_max < a_min:
                return False
            else:
                # y = kx + m
                # ya = k_a * x + m_a
                # yb = k_b * x + m_b
                # ->   k_a *x + m_a - k_b * x - m_b = 0
                # ->   x(k_a - k_b) = m_b - m_a
                # ->   x = (m_b - m_a) / (k_a - k_b)
                
                if not f_eq(a_k - b_k, 0): 
                    x = (b_m - a_m) / (a_k - b_k)
                    if a_min < x < a_max and b_min < b_max:
                        return True
                else:
                    # same derivative of both lines
                    return f_eq(a_m, b_m)
        return False

    def get_boundary_box(self, rotation=0):
        """ Get the [minx, miny, maxX, maxY] for the polygon """
        pts = self.get_cartesian_points(rotation=rotation)
        boundary = [None, None, None, None]
        for p in pts: 
            boundary[0] = p[0] if (boundary[0] is None) or (p[0] < boundary[0]) else boundary[0]
            boundary[1] = p[1] if (boundary[1] is None) or (p[1] < boundary[1]) else boundary[1]

            boundary[2] = p[0] if (boundary[2] is None) or (p[0] > boundary[2]) else boundary[2]
            boundary[3] = p[1] if (boundary[3] is None) or (p[1] > boundary[3]) else boundary[3]
        return boundary


    def print(self):
        print("SimplePoly:")
        print("   color = %s" % self.color)
        print("   fill = %s" % self.fill)
        self.points.print()


class Polygon:
    def __init__(self, screen=None):
        self.polygons = { }
        self.polyOrder = [ ]
        self.poly_visible = { }
        self.screen = screen
        self.rotation = 0.0
        self.translation = [0, 0]

    def move_to(self, p):
        self.translation = [ p[0], p[1] ]

    def move_rel(self, p):
        [ x, y ] = self.get_position()
        x += p[0]
        y += p[1]
        self.move_to([x, y])

    def get_position(self):
        return self.translation

    def get_rotation(self):
        return self.rotation

    def set_rotation(self, rotation):
        self.rotation = rotation % (2 * math.pi)

    def rotate(self, relative_rotation):
        self.set_rotation(self.get_rotation() + relative_rotation)
    
    def add_polygon(self, poly_id, points, color=[255, 255, 255], fill=False, linewidth=1, 
                    visible=True):
        p = SimplePoly(color=color, fill=fill, linewidth=linewidth)
        self.polygons[poly_id] = p
        p.add_cartesianPoints(points)
        self.polyOrder.append(poly_id)
        self.poly_visible[poly_id] = visible

    def set_visibility(self, poly_id, visible=True):
        self.poly_visible[poly_id] = visible 

    def draw(self):
        for p_id in self.polyOrder:
            if self.poly_visible[p_id]:
                p = self.polygons[p_id]
                p.draw(self.screen, self.rotation, self.translation)

    def get_boundary_box(self):
        """ Get the [minx, miny, maxX, maxY] for the polygon """
       
        boundary = None
        p = self.polyOrder

        for p_id in self.polyOrder:
            p = self.polygons[p_id]
        
            if self.poly_visible[p_id]:
                b = p.get_boundary_box(self.get_rotation())
                if boundary is None: 
                    boundary = b
                else:
                    boundary[0] = b[0] if b[0] < boundary[0] else boundary[0]
                    boundary[1] = b[1] if b[1] < boundary[1] else boundary[1]
                    boundary[2] = b[2] if b[2] > boundary[2] else boundary[2]
                    boundary[3] = b[3] if b[3] > boundary[3] else boundary[3]

        pos = self.translation
        boundary[0] += pos[0]
        boundary[1] += pos[1]
        boundary[2] += pos[0]
        boundary[3] += pos[1]
    
        return boundary

    def get_edges(self, translation=[0,0], rotation=0):
        boundary = None
        p = self.polyOrder

        edges = []
        for p_id in self.polyOrder:
            p = self.polygons[p_id]
        
            if self.poly_visible[p_id]:
                pts = p.get_cartesian_points(rotation)
                for n in range(0, len(pts)):
                    p1 = pts[n]
                    p2 = pts[(n + 1) % len(pts)]
                   
                    t_p1 = [ p1[0] + translation[0], 
                             p1[1] + translation[1] ] 
                    t_p2 = [ p2[0] + translation[0], 
                             p2[1] + translation[1] ] 
                    edges.append([t_p1, t_p2])
        return edges

    def print(self):
        print("POLYGON:")
        
        for p_id in self.polyOrder:
            print("  p_id = %s" % p_id)
            p = self.polygons[p_id]
            p.print()

class FloatingObject():
    def __init__(self, polygon = None, speed = [0, 0], rot_speed = 0.0):
        self.polygon = polygon 
        self.speed_vector = speed
        self.rot_speed = rot_speed

        self.speed_vector_cache = { }

        self.collision_polygon = None

    def SetPolygon(self, polygon):
        self.polygon = polygon

    def get_polygon(self):
        return self.polygon

    def get_rotational_speed(self):
        return self.rot_speed

    def get_rotational_angle(self):
        return self.polygon.get_rotation()

    def set_rotational_speed(self, rot_speed):
        self.rot_speed = rot_speed

    def change_rotational_speed(self, delta_speed, clockwise=True, 
                                max_speed=None):
        cur_speed = self.get_rotational_speed()
        cur_speed = cur_speed + (delta_speed if clockwise else -delta_speed)
        if max_speed and cur_speed > max_speed: cur_speed = max_speed
        if max_speed and cur_speed < -max_speed: cur_speed = -max_speed
        self.set_rotational_speed(cur_speed)

    def do_movement(self):
        # change rotation of polygon
        self.polygon.rotate(self.get_rotational_speed())

        sv = self.get_speed_vector()

        dx = 0
        dy = 0
        cache_key = ("%d-%d" % (round(100 * sv[0]),
                                round(100 * sv[1])))
        if cache_key in self.speed_vector_cache:
            # cache hit
            dx = self.speed_vector_cache[cache_key][0] 
            dy = self.speed_vector_cache[cache_key][1] 
        else:
            dx = sv[0] * math.cos(sv[1])
            dy = -sv[0] * math.sin(sv[1])
            self.speed_vector_cache[cache_key] = [ dx, dy ]

        self.move_rel([dx, dy])

    def set_speed_vector(self, v):
        self.speed_vector = v

    def get_speed_vector(self):
        return self.speed_vector

    def invert_speed_vector(self):
        v = self.get_speed_vector()
        v[1] = (v[1] + math.radians(180)) % math.radians(360)

    def draw(self):
        self.polygon.draw()

    def move_to(self, p):
        self.polygon.move_to(p)
        self.get_collision_polygon().move_to(p)

    def move_rel(self, p):
        self.polygon.move_rel(p)
        self.get_collision_polygon().move_rel(p)

    def get_position(self):
        return self.polygon.get_position()

    def get_rotation(self):
        return self.polygon.get_rotation()

    def set_rotation(self, rotation):
        self.polygon.set_rotation(rotation)
        #self.get_collision_polygon().set_rotation(rotation)

    def rotate(self, relative_rotation):
        self.polygon.rotate(relative_rotation)

    def set_collision_polygon(self, cp):
        self.collision_polygon = cp

    def get_collision_polygon(self):
        return self.collision_polygon

    def check_collisions(self, floating_objects):
        # check if any of the floating objects are
        # in collision with self

        cp = self.get_collision_polygon()
        c_box = cp.get_boundary_box()

        c_edges = cp.get_edges(cp.get_position(), self.get_rotation())

        # Edges for ship
        cp_lines = []
        for e in c_edges:
            cp_lines.append(PolarPoints.find_line_equation(e[0], e[1]))

        collision = False
        for o in floating_objects:
            o_cp = o.get_collision_polygon()
            o_box = o_cp.get_boundary_box()

            # Check if the boxes overlap
            if SimplePoly.box_overlap(c_box, o_box):
                o_edges = o_cp.get_edges(o_cp.get_position())

                # Build list of edges in the two polygons 
                # see if any of the edges from poly1 overlap with poly2
                o_lines = []
                for i,e in enumerate(o_edges):
                    o_lines.append(PolarPoints.find_line_equation(e[0], e[1]))

                for o_line in o_lines:
                    for cp_line in cp_lines:
                        if SimplePoly.line_overlap(cp_line, o_line):
                            collision = True
                            break
                    if collision: 
                        break

        return collision 

    def check_off_screen(self):
        """ Check if the object is off screen. 
            Returns true if the object is now off screen, 
            otherwise false """
   
        poly = self.get_polygon() 
        box = poly.get_boundary_box()

        screen_size = settings['screen_size']
    
        if (box[2] < 0 or box[3] < 0 or
            box[0] > screen_size[0] or 
            box[1] > screen_size[1]):
            return True
        else: 
            return False
            
   

pygame.init()
screen = pygame.display.set_mode((settings['screen_size']))
done = False
is_blue = True
x = 30
y = 30

clock = pygame.time.Clock()

def build_ship(screen):
    # Define some simple polygons
    p = Polygon(screen)
    p.add_polygon('ship', [[-30,-5], [-30, 5], [10, 0]], linewidth=0, color=[127, 127, 255])
    #p.add_polygon('triangle', [[0,0], [-10, -20], [10, -20]], fill=True)
    p.add_polygon('down', [[0,0], [0, 10]])
    p.add_polygon('up', [[0,0], [0, -10]])
    p.add_polygon('left', [[0,0], [-10, 0]])
    p.add_polygon('right', [[0,0], [10, 0]], visible=False)
    p.add_polygon('flame_right', [[-15,5], [-10, 4], [-12, 15]], color=[200,150,50], fill=True, visible=False)
    p.add_polygon('flame_left', [[-15,-5], [-10, -4], [-12, -15]], color=[200,150,50], fill=True, visible=False)
    p.add_polygon('flame_rear', [[-15,-5], [-30, 0], [-15, 5]], color=[200,150,50], fill=True, visible=False)

    ship = FloatingObject()
    ship.SetPolygon(p)

    ship.set_rotational_speed(math.radians(0))

    # polygon to keep track of the outline
    cp = Polygon(screen)
    cp.add_polygon('collision', [[-30,-5], [-30, 5], [10, 0]])

    print("Setting collision polygon: ")
    cp.print()

    ship.set_collision_polygon(cp)
    return ship


def build_boulder(screen):
    p = Polygon(screen) 

    # 8 sided "ball"
    sides = 8
    step_angle = 2 * math.pi / sides
    radius = 20

    vertices = []

    for i in range(0, sides):
        a = step_angle * i

        xp = radius * math.cos(a)
        yp = radius * math.sin(a)
        vertices.append([xp, yp])


    print("VERTICES: %s " % vertices)
          
    p.add_polygon('boulder', vertices, fill=True, 
                  color=[127, 127, 127])

    boulder = FloatingObject()
    boulder.SetPolygon(p)
    boulder.set_rotational_speed(math.radians(1.0))

    # polygon to keep track of the outline
    cp = Polygon(screen)
    cp.add_polygon('collision', [[-20,-20], [-20, 20], [20, 20], [20, -20]])
    
    boulder.set_collision_polygon(cp)
    return boulder


ship = build_ship(screen)
ship_poly = ship.get_polygon()


boulders = []
for f in range(0, 5):
    boulder = build_boulder(screen)
    boulder.move_to([random.random()*settings['screen_size'][0], 
                     random.random()*settings['screen_size'][1]])
    boulder.set_speed_vector([random.random(), math.radians(random.random() * 360)])

    boulders.append(boulder)    

ship.move_to([100, 100])


col_count = 0
while not done:
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        done = True
                if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                        is_blue = not is_blue
        
        pressed = pygame.key.get_pressed()

        # Set visibility of flames
        ship_poly.set_visibility('flame_right', False)
        ship_poly.set_visibility('flame_left', False)
        ship_poly.set_visibility('flame_rear', False)

        if pressed[pygame.K_RIGHT]:
            ship.change_rotational_speed(
                settings['delta_rotational_speed'], 
                clockwise=False, 
                max_speed=settings['max_rotational_speed'])
            ship_poly.set_visibility('flame_right', True)
        if pressed[pygame.K_LEFT]: 
            ship.change_rotational_speed(
                settings['delta_rotational_speed'], 
                clockwise=True, 
                max_speed=settings['max_rotational_speed'])
            ship_poly.set_visibility('flame_left', True)

        if pressed[pygame.K_UP]: 
            # Thruster
            sv = ship.get_speed_vector()
            rot = ship.get_rotation()

            # Add the vector pointing at rot with abs(acc) to speed_vector
            acc = settings['thrust_accelleration']
            dv = [ acc, rot ]

            ship.set_speed_vector(PolarPoints.add_polar_points(sv, dv))
            ship_poly.set_visibility('flame_rear', True)

        if pressed[pygame.K_DOWN]: 
            # reset
            ship.set_rotation(math.degrees(0))
            ship.move_to([100, 100])
            ship.set_speed_vector([0, 0])
            ship.set_rotational_speed(0)
 
        screen.fill((0, 0, 0))

        c = ship.check_collisions(boulders)
        if c:
            col_count += 1
            print("  COLLISION  ", col_count)


        for boulder in boulders:
            boulder.do_movement()
            
            boulder.draw()

        ship.do_movement()
        if ship.check_off_screen():
            print("OFF SCREEN")
            ship.invert_speed_vector()
            ship.do_movement()
        ship.draw()

        pygame.display.flip()
        clock.tick(60)

