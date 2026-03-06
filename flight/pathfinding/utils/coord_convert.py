import numpy as np

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
    
    def sim_to_real_convert(self, sim_coords:tuple[int,int]):
        segment_size_12 = [(self.corner_coord_2[0] - self.corner_coord_1[0])/self.sim_sides[0], (self.corner_coord_2[1] - self.corner_coord_1[1])/self.sim_sides[0]]
        segment_size_34 = [(self.corner_coord_4[0] - self.corner_coord_3[0])/self.sim_sides[0], (self.corner_coord_4[1] - self.corner_coord_3[1])/self.sim_sides[0]]
        segment_size_31 = [(self.corner_coord_1[0] - self.corner_coord_3[0])/self.sim_sides[0], (self.corner_coord_1[1] - self.corner_coord_3[1])/self.sim_sides[0]]
        segment_size_42 = [(self.corner_coord_2[0] - self.corner_coord_4[0])/self.sim_sides[0], (self.corner_coord_2[1] - self.corner_coord_4[1])/self.sim_sides[0]]

        slider_point12 = [self.corner_coord_1[0]+sim_coords[0]*segment_size_12,self.corner_coord_1[1]+sim_coords[0]*segment_size_12]
        slider_point34 = [self.corner_coord_3[0]+sim_coords[0]*segment_size_34,self.corner_coord_3[1]+sim_coords[0]*segment_size_34]
        slider_point31 = [self.corner_coord_3[0]+sim_coords[1]*segment_size_31,self.corner_coord_3[1]+sim_coords[1]*segment_size_31]
        slider_point42 = [self.corner_coord_4[0]+sim_coords[1]*segment_size_42,self.corner_coord_4[1]+sim_coords[1]*segment_size_42]

        final_lon = ((slider_point12[0]*slider_point34[1]-slider_point12[1]*slider_point34[0])*(slider_point31[0]-slider_point42[0])-(slider_point12[0]-slider_point34[0])*(slider_point31[0]*slider_point42[1]-slider_point31[0]*slider_point42[0]))/((slider_point12[0]-slider_point34[0])*(slider_point31[1]-slider_point42[1])-(slider_point12[1]-slider_point34[1])*(slider_point31[0]-slider_point42[0]))
        final_lat = ((slider_point12[0]*slider_point34[1]-slider_point12[1]*slider_point34[0])*(slider_point31[1]-slider_point42[1])-(slider_point12[1]-slider_point34[1])*(slider_point31[0]*slider_point42[1]-slider_point31[0]*slider_point42[0]))/((slider_point12[0]-slider_point34[0])*(slider_point31[1]-slider_point42[1])-(slider_point12[1]-slider_point34[1])*(slider_point31[0]-slider_point42[0]))

        return final_lon, final_lat


'''
Coord convert test case: My old schools middleschool football field.

1: 36°01'18.06"N 95°56'30.59"W or 36.021683, -95.941831.
2: 36°01'14.50"N 95°56'30.68"W or 36.020694, -95.941856.
3: 36°01'18.10"N 95°56'32.54"W or 36.021694, -95.942372
4: 36°01'14.53"N 95°56'32.63"W or 36.020703, -95.942397
Field Dimensions: ~360 ft x ~520 ft

Feel free to tweak these coords to be more even or more or less square, 
just be sure to note when, why, and what happened regarding the change.

https://earth.google.com/earth/d/1xIViZX3BNUBbDvmUx16SDTol8_dzGt-L?usp=sharing
'''
def run_tests():
    lat_lon1 = [input("lat 1: "), input("lon 1: ")]
    lat_lon2 = [input("lat 2: "), input("lon 2: ")]
    lat_lon3 = [input("lat 3: "), input("lon 3: ")]
    lat_lon4 = [input("lat 4: "), input("lon 4: ")]

    sim_sides = [np.sqrt(), np.sqrt()]

    coord_converter = SimToLatLonTransformer([lat_lon1, lat_lon2, lat_lon3, lat_lon4], sim_sides)

    x = input("Sim X:")
    y = input("Sim y:")

    print(coord_converter.sim_to_real_convert([x,y]))