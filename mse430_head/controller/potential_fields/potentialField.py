import numpy as np
import math
from time import sleep

class PotentialField:

    def __init__(self, location, radius, spread, strength, field_type='repulsor', orient='clock', const_strength = 5):
        self.location = location
        self.radius = radius
        self.spread = spread
        self.field_type = field_type
        self.field_strength = strength
        self.infinity = 10
        self.orient = orient
        self.const_strength = const_strength

    def distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def effect(self, location):
        if self.field_type == 'repulsor':
            d = self.distance(location, self.location)
            if d > self.radius + self.spread:
                return [0, 0]
            theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0]) + np.pi/180
            #  a constant to give them some angle
            if d > self.radius:
                return [-self.const_strength*math.cos(theta),
                        -self.const_strength * math.sin(theta)]
            else:
                return [-np.sign(math.cos(theta))*self.infinity, -np.sign(math.sin(theta))*self.infinity]
        elif self.field_type == 'tangent':
            d = self.distance(location, self.location)
            if d > self.spread:
                return [0, 0]
            #  the constant term on the end gives the tangent angle minus a little to push outward as well
            if self.orient == 'counter':
                theta = math.atan2(-self.location[1] + location[1], -self.location[0] + location[0])
            elif self.orient == 'clock':
                theta = math.atan2(-self.location[1] + location[1], -self.location[0] + location[0])
            else:
                raise ValueError("No correct orientation!")
            if d >= self.radius:
                x_tan = self.field_strength * math.cos(theta + np.pi/2)
                x_rep = -self.const_strength*math.cos(theta)
                y_tan = self.field_strength * math.sin(theta - np.pi/2)
                y_rep = -self.const_strength * math.sin(theta)
                return [x_tan + x_rep, y_tan + y_rep]
            else:
                return [-np.sign(math.cos(theta)) * self.infinity, -np.sign(math.sin(theta)) * self.infinity]
        elif self.field_type == 'attractor':
            d = self.distance(location, self.location)
            if d > self.radius + self.spread:  # if it is outside of the potential field's effect
                return [0, 0]
            theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0]) + np.pi/180
            #  a constant to give them some angle
            if d > self.radius: # within the radius of effect, but outside the object's own radius
                return [self.const_strength * math.cos(theta),
                        self.const_strength * math.sin(theta)]
            else: # 'On top' of the object
                return [self.field_strength*(d + 2)*math.cos(theta),
                        self.field_strength * (d + 2) * math.sin(theta)]
    def o_effect(self, location):
        if self.field_type == 'repulsor':
            d = self.distance(location, self.location)
            if d > self.radius + self.spread:
                return [0, 0]
            theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0])
            #  a constant to give them some angle
            if d >= self.radius:
                return [-self.field_strength*(self.spread + self.radius - d)*math.cos(theta),
                        -self.field_strength * (self.spread + self.radius - d) * math.sin(theta)]
            else:
                return [-np.sign(math.cos(theta))*self.infinity, -np.sign(math.sin(theta))*self.infinity]
        elif self.field_type == 'tangent':
            d = self.distance(location, self.location)
            if self.orient == 'counter':
                theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0]) + np.pi/4
            elif self.orient == 'clock':
                theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0]) - np.pi/4
            else:
                raise ValueError("No correct orientation!")
            if d > self.radius + self.spread:
                return [0, 0]
            #  the constant term on the end gives the tangent angle minus a little to push outward as well
            if d > self.radius + 1:
                return [-self.field_strength * (self.spread + self.radius - d) * math.cos(theta),
                        -self.field_strength * (self.spread + self.radius - d) * math.sin(theta)]
            else:
                return [-np.sign(math.cos(theta)) * self.infinity, -np.sign(math.sin(theta)) * self.infinity] 
        elif self.field_type == 'attractor':
            d = self.distance(location, self.location)
            theta = math.atan2(self.location[1] - location[1], self.location[0] - location[0])
            if d > self.radius + self.spread:  
                return [self.field_strength * self.spread * math.cos(theta), 
                        self.field_strength * self.spread * math.sin(theta)]
            if d >= self.radius: 
                return [self.field_strength * (d - self.radius) * math.cos(theta),
                        self.field_strength * (d - self.radius) * math.sin(theta)]
            else: # 'On top' of the object
                return [0, 0]
