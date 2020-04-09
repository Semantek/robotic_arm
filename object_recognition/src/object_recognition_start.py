#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('object_recognition')
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from object_recognition.msg import RecognizedObjects
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

PREFIX = '_'

# Creating dictionary between PointField types and numpy types
type_felds_list = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')),(PointField.UINT32, np.dtype('uint32')), (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
feld_dict = dict(type_felds_list)

# sizes (in bytes) of PointField types
type_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2, PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


# Extracting points coordinates from a data array
def get_points(data_array, remove_other=True, dtype=np.float):
	# Select axis
	if remove_other:
		mask = np.isfinite(data_array['x']) & np.isfinite(data_array['y']) & np.isfinite(data_array['z'])
		data_array = data_array[mask]
	# Pull out x, y, and z values
	points = np.zeros(list(data_array.shape) + [3], dtype=dtype)
	points[...,0] = data_array['x']
	points[...,1] = data_array['y']
	points[...,2] = data_array['z']

	return points

def grab_rgb_image(imput_arr, hight, width):
	#Copy RGB part from a camera array
	rgb_arr = imput_arr['rgb'].copy()
	#Set type
	rgb_arr.dtype = np.uint32
	#Convert to 8bit colors from format 32bit: 00RRGGBB
	red = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
	green = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
	blue = np.asarray(rgb_arr & 255, dtype=np.uint8)
	#Reshape 1D arrays to hightXwidth matrix
	np.reshape(red, (hight, width))
	np.reshape(green, (hight, width))
	np.reshape(blue, (hight, width))
	#Merge tree matrixes in one
	image = np.dstack((red,green,blue))

	return image


def extract_dtype_list(msg):
	offset = 0
	new_dtype_list = []
	for f in msg.fields:
		while offset < f.offset:
		# If are extra padding between fields
			new_dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
			offset += 1
		new_dtype_list.append((f.name, feld_dict[f.datatype]))
		offset += type_sizes[f.datatype]

	# If are extra padding between points
	while offset < msg.point_step:
		new_dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
		offset += 1   
	#print(new_dtype_list)
	return new_dtype_list

def find_XYZ_datatype(msg):
	dtype = ''
	for f in msg.fields:
		if f.name == 'x':
			dtype = feld_dict[f.datatype]
	return dtype


#Transformation of camera pixel coordinates to global world coordinates
def coordinate_transformation (camera_pos, camera_points_XYZ):
	roll = camera_pos[3]
	pitch = camera_pos[4]
	yaw = camera_pos[5]
	camera_xyz = [camera_pos[0],camera_pos[1],camera_pos[2]]
	out_points = []
	#Set transform matrix
	Mr = [[1.,0.,0.],
		[0.,np.cos(roll), -np.sin(roll)],
		[0.,np.sin(roll), np.cos(roll)]]
	Mp = [[np.cos(pitch), 0., np.sin(pitch)],
		[0.,1.,0.],        
		[np.sin(pitch),0., np.cos(pitch)]]
	My = [[np.cos(yaw), -np.sin(yaw),0.],
		[np.sin(yaw), np.cos(yaw), 0.],
		[0.,0.,1.]]
	M = np.dot(Mr,np.dot(Mp,My))
	#Coordonate of the Point in our world: x = z_cam, y = -x_cam, z = -y_cam
	for i in range (len(camera_points_XYZ)):
		vec = [camera_points_XYZ[i][2], -camera_points_XYZ[i][0], -camera_points_XYZ[i][1]]
		out_points.append(camera_xyz + np.dot(M,vec))
	return out_points

def convert_to_pose(point_XYZ):
	return Pose(position=Point(x=point_XYZ[0],y=point_XYZ[1],z=point_XYZ[2]), orientation=Quaternion(x=0., y=0., z=0., w=0.))

# Define main Class
class main_loop:

	def __init__(self, no_view):
			
		self.detected_contours = []
		self.points = []
		self.no_view = no_view
		#Subscriber to the camera depth
		self.image_sub_depth = rospy.Subscriber("/camera/depth/points",PointCloud2,self.callback_depth, queue_size = 10)
		# Coordinates of object publisher  
		self.objects_pub = rospy.Publisher("recognized_objects", PoseArray, queue_size=10)


	def callback_depth(self,msg):
		# construct a numpy record type equivalent to the point type of this cloud
		data_type_list = extract_dtype_list(msg)
	
		# parse the cloud into an array
		cloud_arr = np.fromstring(msg.data, data_type_list)
		cloud_arr = cloud_arr[[fname for fname, _type in data_type_list if not (fname[:len(PREFIX)] == PREFIX)]]
		dtype = find_XYZ_datatype(msg)
		cloud_arr = np.reshape(cloud_arr, (msg.height, msg.width))
		cv_image = grab_rgb_image (cloud_arr, 640, 480)
		#vis = cv.CreateMat(480, 640, cv.CV_8UC3)
		#cv2.imshow("Camera image", cv_image)
		self.points = get_points(cloud_arr, dtype=dtype)

		# Color image to grayscale	
		cv_image_temp = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		cv_image_temp = cv2.GaussianBlur(cv_image_temp, (5, 5), cv2.BORDER_DEFAULT)
		# Binarization
		ret,cv_image_temp = cv2.threshold(cv_image_temp, 20, 255, cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
		#Contours finding

		
		self.detected_contours = cv2.findContours(cv_image_temp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    		#Count up coordinates
    		coor = []
		width = cv_image_temp.shape[1]
		hight = cv_image_temp.shape[0]

		for c in self.detected_contours:
      			x_mean = .0
 			y_mean = .0
 			z_mean = .0
			for i in range (len(c)):
				index = c[i][0][1]*width + c[i][0][0]

        			x_mean += self.points[index][0]
        			y_mean += self.points[index][1]
        			z_mean += self.points[index][2]
			x_mean /= len(c)
			y_mean /= len(c)
			z_mean /= len(c)
			coor.append([x_mean,y_mean,z_mean])


		# Perform the distance transform algorithm
		#cv_image_2 = cv2.distanceTransform(cv_image_temp, cv2.DIST_L2, 3)
		border = 20
		img1 = cv_image.copy()
		objects_coor = []
		for c in self.detected_contours:
			#cv2.drawContours(img1, [c], -1, (0,255,0), 3)
			# compute the center of the contour
			M = cv2.moments(c)
			area = cv2.contourArea(c)
			flag = False
			for i in range (len(c)):
			#Filtred countors, that partial out of camera view
				if (c[i][0][0] < border) or (c[i][0][0] > width-border) or (c[i][0][1] < border) or (c[i][0][1] > hight-border):
					flag = True
			if (M["m00"] != 0) and (area > 100) and (area < 50000) and not flag:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				#cv2.circle(img1, (cX, cY), 4, (255, 255, 255), -1)
				index = cY*width + cY
				cv2.drawContours(img1, [c], -1, (0,255,0), 3)
				x_mean = .0
 				y_mean = .0
 				z_mean = .0
				for i in range (len(c)):
					index = c[i][0][1]*width + c[i][0][0]

        				x_mean += self.points[index][0]
        				y_mean += self.points[index][1]
        				z_mean += self.points[index][2]
				x_mean /= len(c)
				y_mean /= len(c)
				z_mean /= len(c)
				delta_z = .0
				for i in range (len(c)):
					index = c[i][0][1]*width + c[i][0][0]
					d = abs(z_mean - self.points[index][2])
					if d < 1. :
						delta_z += d
				delta_z /= len(c)
				if delta_z > 0.07:
					x1_mean = .0
 					y1_mean = .0
 					z1_mean = .0
					x2_mean = .0
 					y2_mean = .0
 					z2_mean = .0
					iter_1 = 0
					iter_2 = 0
					for i in range (len(c)):
						index = c[i][0][1]*width + c[i][0][0]
						if self.points[index][2] > z_mean:
        						x1_mean += self.points[index][0]
        						y1_mean += self.points[index][1]
        						z1_mean += self.points[index][2]
							iter_1 += 1
						else:
        						x2_mean += self.points[index][0]
        						y2_mean += self.points[index][1]
        						z2_mean += self.points[index][2]
							iter_2 += 1
					x1_mean /= iter_1
					y1_mean /= iter_1
					z1_mean /= iter_1
					x2_mean /= iter_2
					y2_mean /= iter_2
					z2_mean /= iter_2
					if iter_1 > 10:
						objects_coor.append([x1_mean, y1_mean, z1_mean])
					if iter_2 > 10:
						objects_coor.append([x2_mean, y2_mean, z2_mean])
				else:
					objects_coor.append([x_mean, y_mean, z_mean])



		if not self.no_view:		
			cv2.imshow("Camera image", img1)
			cv2.waitKey(5)
		#Set position of the depth camera TODO auto
		camera_position = [1.33, 0. , 1.2, 0. , 0., np.pi ]
		#Transform points 3D coordinate
		objects_coor = coordinate_transformation (camera_position, objects_coor)
		#Create message
		points_to_poses = []
		for p in objects_coor:
			points_to_poses.append(convert_to_pose(p))
		pub_msg = PoseArray(header=Header(stamp=rospy.Time.now(), frame_id="map"), poses=points_to_poses) 
		#Print coordinate
		print ("Found ",len(objects_coor), "objects")
		for c in objects_coor:
			print (c)
		self.objects_pub.publish(pub_msg)

#--------------- MAIN LOOP
def main(args):
	#--- Create the object
	no_view = False
	if args[1] == 'true':
		no_view = True
	loop = main_loop(no_view)

	#--- Initialize the ROS node
	rospy.init_node('object_rec', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	
    
	#--- Close all cv windows
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
