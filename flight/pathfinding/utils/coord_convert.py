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
    
    def convert_coords(self, sim_coords:tuple[int,int]):
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
    
