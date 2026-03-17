import numpy as np
import math

class SimToLatLonTransformer:
    def __init__(self, corner_coords:tuple[tuple[float,float]], sim_side_lengths:tuple[int,int]):
        self.sim_sides = sim_side_lengths

        '''
        1----------2
        |          |
        |          |
        3----------4
        '''

        self.corner_coord_1 = corner_coords[0]
        self.corner_coord_2 = corner_coords[1]
        self.corner_coord_3 = corner_coords[2]
        self.corner_coord_4 = corner_coords[3]

        self.scale = 1000 # sim to real
        self.origin = self.corner_coord_3
        self.offset_angle = -math.atan2(self.corner_coord_4[1]-self.corner_coord_3[1], self.corner_coord_4[0]-self.corner_coord_3[0]) # Real to sim CCW
        angle = math.acos((np.subtract(self.corner_coord_4, self.corner_coord_3)@np.subtract(self.corner_coord_1, self.corner_coord_3)))
        if angle > math.pi/2:
            one_three_side_length = math.sqrt((self.corner_coord_1[0]-self.corner_coord_3[0])**2 + (self.corner_coord_1[0]-self.corner_coord_3[0])**2)
            three_four_side_length = math.sqrt((self.corner_coord_4[0]-self.corner_coord_3[0])**2 + (self.corner_coord_4[0]-self.corner_coord_3[0])**2)
            offset_length = one_three_side_length*math.cos(angle)
            origin_transform_vector = (-offset_length*(self.corner_coord_4[0]-self.corner_coord_3[0])/three_four_side_length,-offset_length*(self.corner_coord_4[0]-self.corner_coord_3[0])/three_four_side_length)
            self.origin = (self.origin[0]+origin_transform_vector[0],self.origin[1]+origin_transform_vector[1])

        print(angle)
    
    def sim_to_real_convert(self, sim_coord:tuple[int,int]):
        segment_size_12 = [(self.corner_coord_2[0] - self.corner_coord_1[0])/self.sim_sides[0], (self.corner_coord_2[1] - self.corner_coord_1[1])/self.sim_sides[0]]
        segment_size_34 = [(self.corner_coord_4[0] - self.corner_coord_3[0])/self.sim_sides[0], (self.corner_coord_4[1] - self.corner_coord_3[1])/self.sim_sides[0]]
        segment_size_31 = [(self.corner_coord_1[0] - self.corner_coord_3[0])/self.sim_sides[1], (self.corner_coord_1[1] - self.corner_coord_3[1])/self.sim_sides[1]]
        segment_size_42 = [(self.corner_coord_2[0] - self.corner_coord_4[0])/self.sim_sides[1], (self.corner_coord_2[1] - self.corner_coord_4[1])/self.sim_sides[1]]

        slider_point12 = [(self.corner_coord_1[0] + sim_coord[0]*segment_size_12[0]),(self.corner_coord_1[1] + sim_coord[0]*segment_size_12[1])]
        slider_point34 = [(self.corner_coord_3[0] + sim_coord[0]*segment_size_34[0]),(self.corner_coord_3[1] + sim_coord[0]*segment_size_34[1])]
        slider_point31 = [(self.corner_coord_3[0] + sim_coord[1]*segment_size_31[0]),(self.corner_coord_3[1] + sim_coord[1]*segment_size_31[1])]
        slider_point42 = [(self.corner_coord_4[0] + sim_coord[1]*segment_size_42[0]),(self.corner_coord_4[1] + sim_coord[1]*segment_size_42[1])]

        x1,y1 = slider_point31
        x2,y2 = slider_point42
        x3,y3 = slider_point34
        x4,y4 = slider_point12

        final_lon = (((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)))
        final_lat = (((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)))

        return final_lon, final_lat
    
    def latlon_to_local(self, lat, lon):
        translated_point = (lat-self.origin[0],lon-self.origin[1])
        rot_mat = [[math.cos(self.offset_angle), -math.sin(self.offset_angle)],
                   [math.sin(self.offset_angle), math.cos(self.offset_angle)]]
        rotated_point = np.matmul(rot_mat, translated_point)
        scaled_point = (rotated_point[0]*self.scale, rotated_point[1]*self.scale)
        return scaled_point
    
    def local_to_latlon(self, x, y):
        scaled_point = (x/self.scale, y/self.scale)
        rot_mat = [[math.cos(self.offset_angle), math.sin(self.offset_angle)],
                   [-math.sin(self.offset_angle), math.cos(self.offset_angle)]]
        rotated_point = np.matmul(rot_mat, scaled_point)
        translated_point = (rotated_point[0]+self.origin[0], rotated_point[1]+self.origin[1])
        return translated_point
    
    


'''
Coord convert test case: My old schools middleschool football field.

1: 36°01'18.06"N 95°56'30.59"W or 36.021683, -95.941831 *
2: 36°01'14.50"N 95°56'30.68"W or 36.020694, -95.941856 **
3: 36°01'18.10"N 95°56'32.54"W or 36.021694, -95.942372 ***
4: 36°01'14.53"N 95°56'32.63"W or 36.020703, -95.942397 ****
Field Dimensions: ~360 ft x ~160 ft $

Test Arbitrary Coord: 100, 100 $$

Feel free to tweak these coords to be more even or more or less square, 
just be sure to note when, why, and what happened regarding the change.

https://earth.google.com/earth/d/1xIViZX3BNUBbDvmUx16SDTol8_dzGt-L?usp=sharing

Shell Commands:
import flight.pathfinding.utils.coord_convert as cc
cc.run_tests()
'''
def run_tests():
    lat_lon1 = [-95.941831, 36.021683] # *
    lat_lon2 = [-95.941856, 36.020694] # **
    lat_lon3 = [-95.942372, 36.021694] # ***
    lat_lon4 = [-95.942397, 36.020703] # ****

    sim_sides = [360, 160] # $

    coord_converter = SimToLatLonTransformer([lat_lon1, lat_lon2, lat_lon3, lat_lon4], sim_sides)

    x = 100 # $$
    y = 100 # $$

    final_lat, final_lon = coord_converter.local_to_latlon(x,y)
    final_lon2, final_lat2 = coord_converter.sim_to_real_convert((x,y))
    x1, y1 = coord_converter.latlon_to_local(final_lat,final_lon)
    print(final_lat)
    print(final_lon)
    print(x1, y1)

if __name__ == "__main__":
    run_tests()