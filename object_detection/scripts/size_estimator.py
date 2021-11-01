#!/usr/bin/env python

from math import sin, radians

class SizeEstimator():
    fov_horizontal = 110.0                          # Degrees
    fov_vertical = 70.0                             # Degrees
    focal_length = 2.12                             # mm
    max_width = 1344                                # pxl
    max_height = 376                                # pxl
    angles_pr_pxl_hor = fov_horizontal/max_width
    angles_pr_pxl_ver = fov_vertical/max_height
    use_single_lense = False

    def main(self, bbox):
        """
        Takes in data from a bounding box message and calculates the size of the object.
        
        Args:
            bbox                        the boundingbox message which includes data needed to calculate the size of an object

        Returns:
            size_estimator_data: List of usefull data
                length_x_mtr: Width of object
                length_y_mtr: Height of object
                redefined_angle_x: The horizontal angle to centre of boundingbox in a new coordinate frame shown below
                redefined_angle_y: The vertical angle to centre of boundingbox in a new coordinate frame shown below

        xy \t xy = -+ \t ++ \n
        xy \t xy = -- \t +-\n

        """
        # Get x-pos of boundingbox
        min_x_pxl = bbox.xmin
        max_x_pxl = bbox.xmax

        # Get y-pos of boundingbox
        # Unintuitively position is logged as top to bottom. We fix it so it is from bot to top
        min_y_pxl = self.max_height - bbox.ymax
        max_y_pxl = self.max_height - bbox.ymin
        
        # Get depth measurement of boundingbox
        depth_mtr = bbox.z

        # Calculate the angles for x and y
        delta_angle_x, angle_centre_object_x = self.calc_angles(self.angles_pr_pxl_hor, min_x_pxl,max_x_pxl) # Note no difference from double lense since fov and max pixels are both halved.
        delta_angle_y, angle_centre_object_y = self.calc_angles(self.angles_pr_pxl_ver, min_y_pxl,max_y_pxl)

        # Calculate the sizes in metres of a boundingbox
        length_x_mtr = self.calc_size(delta_angle_x, depth_mtr)
        length_y_mtr = self.calc_size(delta_angle_y, depth_mtr)
        # Redefine angles to coord frame
        redefined_angle_x = self.redefine_angles(angle_centre_object_x, self.fov_horizontal)
        redefined_angle_y = self.redefine_angles(angle_centre_object_y, self.fov_vertical)

        if self.use_single_lense:
            redefined_angle_x = self.redefine_angles(angle_centre_object_x, self.fov_horizontal*0.5)  

        size_estimator_data = [length_x_mtr, length_y_mtr, redefined_angle_x, redefined_angle_y]
        return size_estimator_data
    
    def redefine_angles(self, angle_centre_object, fov_type):
        """
        Creates a coordinate system for the view. Angles in the right half of the view are positive, and the left half are negative.
        Does the same for top and bottom half of screen. Sets Origo in the center of the screen.

        Args:
            angle_centre_object         angle_centre_object_x/angle_centre_object_y
            fov_type                    fov_horizontal/fov_vertical

        Returns:
            redefined_angle             angle in new coordinate system, pos angle in right/upper half os the screen and vice versa
        """

        # Split the screen in two halves
        new_fov = fov_type * 0.5
        
        if angle_centre_object >= new_fov:
            redefined_angle = angle_centre_object - new_fov
        else:
            redefined_angle = (-1)*(new_fov - angle_centre_object)

        return redefined_angle
        
    def calc_size(self, delta_angle, dist_to_object):
        """
        Calculate angles of bounding box width/height.
          
        Args:
            delta_angle             The angle between max/min position
            dist_to_object          The distance to the object in the bounding box
        Returns:
            length_in_mtr           The width or height of the detected object box
        """

        length_in_mtr = dist_to_object * sin(radians(delta_angle))

        return length_in_mtr

    def calc_angles(self, angles_pr_pxl, min_pos, max_pos):
        """
        Calculate angles of bounding box width/height.
          
        Args:
            angles_pr_pxl   angles_pr_pxl_hor/angles_pr_pxl_ver
            min_pos         minimum pxl position to calculate angle
            max_pos         maximum pxl position to calculate angle

        Returns:
            delta_angle             The angle between max/min position
            angle_centre_object     The angle to the centre of the object in 1 dimension
        """

        angle_max = max_pos * angles_pr_pxl
        angle_min = min_pos * angles_pr_pxl
        delta_angle = angle_max - angle_min
        angle_centre_object = angle_min + delta_angle * 0.5
        
        return delta_angle, angle_centre_object
    
    


