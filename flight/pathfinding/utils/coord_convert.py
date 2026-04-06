import numpy as np
import math

class SimToLatLonTransformer:
    def __init__(self, corner_coords:tuple[tuple[float,float]], sim_width:float):



        '''
        If angle 134 is acute, the local origin is placed at point 3, otherwise it is put on the projected line that comes from 34 and is directly below point 1. This ensures that the entire minefield is in the first quadrant
        1----------2
        |          |
        |          |
        o3---------4
        '''

        '''
        1-------------2
        |\\            |
        | \\           |
        o--3----------4
        '''

        '''
           1-------------2
          /              |
         /               |
        o3---------------4
        '''

        # Using corner 3 as origin (0,0) in sim
        self.origin_lat, self.origin_lon = corner_coords[2]
        self.sim_w = sim_width

        # Meters per degree at corner 3 latitude and longitude
        self.m_per_lat = 111320.0
        self.m_per_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        # Convert 1-3 and 3-4 to meters from degrees
        c1_lat, c1_lon = corner_coords[0]
        c4_lat, c4_lon = corner_coords[3]

        # Vector components from point 3 to point 4
        base_x = (c4_lon - self.origin_lon) * self.m_per_lon
        base_y = (c4_lat - self.origin_lat) * self.m_per_lat

        # Vector components from point 3 to point 1
        dx13 = (c1_lon - self.origin_lon) * self.m_per_lon
        dy13 = (c1_lat - self.origin_lat) * self.m_per_lat

        # Find angle 134
        self.angle_134 = math.acos(np.dot([base_x, base_y], [dx13, dy13])/(math.sqrt(base_x**2+base_y**2)*math.sqrt(dx13**2+dy13**2)))

        # Move origin if angle 134 is obtuse
        if self.angle_134 > math.pi/2:
            one_three_side_length = math.sqrt(dx13**2 + dy13**2)
            three_four_side_length = math.sqrt(base_x**2 + base_y**2)
            offset_length = one_three_side_length*math.cos(self.angle_134)

            # Project vector from 3 to 1 onto base
            origin_transform_vector = (offset_length*(base_x)/three_four_side_length,offset_length*(base_y)/three_four_side_length)

            # Move origin
            self.origin_lat = self.origin_lat + (origin_transform_vector[1] / self.m_per_lat)
            self.origin_lon = self.origin_lon + (origin_transform_vector[0] / self.m_per_lon)

            # Update global coords relative to moved origin
            base_x = (c4_lon - self.origin_lon) * self.m_per_lon
            base_y = (c4_lat - self.origin_lat) * self.m_per_lat
            dx13 = (c1_lon - self.origin_lon) * self.m_per_lon
            dy13 = (c1_lat - self.origin_lat) * self.m_per_lat
        
        # Angle of the bottom edge (3-4) relative to East
        self.offset_angle = math.atan2(base_y, base_x)
        
        # Scale: pixels/units per meter
        real_dist = math.sqrt(base_x**2 + base_y**2)
        self.scale = self.sim_w / real_dist

    def latlon_to_local(self, lat, lon):
        # 1. Convert to relative meters
        dy = (lat - self.origin_lat) * self.m_per_lat
        dx = (lon - self.origin_lon) * self.m_per_lon
        
        # 2. Rotate to align with sim axes
        # We rotate by -offset_angle to bring the "3-4 edge" to the X-axis
        c, s = math.cos(-self.offset_angle), math.sin(-self.offset_angle)
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        
        # 3. Scale to sim units
        return [rx * self.scale, ry * self.scale]

    def local_to_latlon(self, x, y):
        # 1. Unscale
        ux, uy = x / self.scale, y / self.scale
        
        # 2. Inverse Rotate
        c, s = math.cos(self.offset_angle), math.sin(self.offset_angle)
        dx = ux * c - uy * s
        dy = ux * s + uy * c
        
        # 3. Convert meters back to Lat/Lon
        lat = self.origin_lat + (dy / self.m_per_lat)
        lon = self.origin_lon + (dx / self.m_per_lon)
        return [lat, lon]


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
    lat_lon1 = [36.021683, -95.941831] # *
    lat_lon1_alt = [36.021695, -95.941831] # *
    lat_lon2 = [36.020694, -95.941856] # **
    lat_lon3 = [36.021694, -95.942372] # ***
    lat_lon4 = [36.020703, -95.942397] # ****

    prop_lat_lon1 = [36.021492, -95.942263]
    prop_lat_lon2 = [36.021144, -95.942249]
    prop_lat_lon3 = [36.020903, -95.941913]

    # Approximate, can't directly select chosen points as measurement points, can only click as near to the point as possible, so should be close but may be slightly off
    act_len_12 = 38.57
    act_len_23 = 40.47
    act_len_13 = 72.57

    test_point = [36.021411, -95.942039]

    sim_sides = [360, 160] # $

    print("Test 1: origin at point 134")
    coord_converter = SimToLatLonTransformer([lat_lon1, lat_lon2, lat_lon3, lat_lon4], sim_sides[0])

    print(coord_converter.angle_134 - math.pi/2) # If positive, origin is not at point 134
    print(coord_converter.latlon_to_local(*test_point)) # Should be close to (100,100)
    print(coord_converter.latlon_to_local(*lat_lon1)) # Should be close to (0, 160)
    print(coord_converter.latlon_to_local(*lat_lon2)) # Should be close to (360, 160)
    print(coord_converter.latlon_to_local(*lat_lon3)) # Should be close to (0, 0)
    print(coord_converter.latlon_to_local(*lat_lon4)) # Should be close to (360, 0)
    print(test_point)
    print(coord_converter.local_to_latlon(*coord_converter.latlon_to_local(*test_point))) # Should match line above
    print()

    print("Test 2: moved origin (These values will be farther from ideal values, because point 1 has been moved to make angle 134 obtuse)")
    coord_converter2 = SimToLatLonTransformer([lat_lon1_alt, lat_lon2, lat_lon3, lat_lon4], sim_sides[0])

    print(coord_converter2.angle_134 - math.pi/2) # If positive, origin is not at point 134
    print(coord_converter2.latlon_to_local(*test_point)) # Should be close to (100,100)
    print(coord_converter2.latlon_to_local(*lat_lon1_alt)) # Should be close to (0, 160)
    print(coord_converter2.latlon_to_local(*lat_lon2)) # Should be close to (360, 160)
    print(coord_converter2.latlon_to_local(*lat_lon3)) # Should be close to (0, 0), x coordinate should be positive
    print(coord_converter2.latlon_to_local(*lat_lon4)) # Should be close to (360, 0)
    print(test_point)
    print(coord_converter2.local_to_latlon(*coord_converter2.latlon_to_local(*test_point))) # Should match line above
    print()


    print("Test 3: Proportion test points unmoved origin")
    test_3_point1 = coord_converter.latlon_to_local(*prop_lat_lon1)
    test_3_point2 = coord_converter.latlon_to_local(*prop_lat_lon2)
    test_3_point3 = coord_converter.latlon_to_local(*prop_lat_lon3)
    test_3_local_dist12 = math.sqrt((test_3_point1[0]-test_3_point2[0])**2 + (test_3_point1[1]-test_3_point2[1])**2)
    test_3_local_dist23 = math.sqrt((test_3_point2[0]-test_3_point3[0])**2 + (test_3_point2[1]-test_3_point3[1])**2)
    test_3_local_dist13 = math.sqrt((test_3_point1[0]-test_3_point3[0])**2 + (test_3_point1[1]-test_3_point3[1])**2)

    #Each of these should be almost exactly the same
    print(act_len_12 / test_3_local_dist12)
    print(act_len_23 / test_3_local_dist23)
    print(act_len_13 / test_3_local_dist13)
    print()

    print("Test 4: Proportion test points moved origin")
    test_4_point1 = coord_converter2.latlon_to_local(*prop_lat_lon1)
    test_4_point2 = coord_converter2.latlon_to_local(*prop_lat_lon2)
    test_4_point3 = coord_converter2.latlon_to_local(*prop_lat_lon3)
    test_4_local_dist12 = math.sqrt((test_4_point1[0]-test_4_point2[0])**2 + (test_4_point1[1]-test_4_point2[1])**2)
    test_4_local_dist23 = math.sqrt((test_4_point2[0]-test_4_point3[0])**2 + (test_4_point2[1]-test_4_point3[1])**2)
    test_4_local_dist13 = math.sqrt((test_4_point1[0]-test_4_point3[0])**2 + (test_4_point1[1]-test_4_point3[1])**2)

    #Each of these should be almost exactly the same
    print(act_len_12 / test_4_local_dist12)
    print(act_len_23 / test_4_local_dist23)
    print(act_len_13 / test_4_local_dist13)


if __name__ == "__main__":
    run_tests()