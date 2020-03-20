#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('object_recognition')
import sys
import rospy
import cv2
import numpy as np
#import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from object_recognition.msg import RecognizedObjects
from cv_bridge import CvBridge, CvBridgeError

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
	#print(data_array['z'])
	# Pull out x, y, and z values
	#points = np.zeros(data_array.shape + (3,), dtype=dtype)
	points = np.zeros(list(data_array.shape) + [3], dtype=dtype)
	points[...,0] = data_array['x']
	points[...,1] = data_array['y']
	points[...,2] = data_array['z']

	return points

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

# Define main Class
class main_loop:

	def __init__(self):
			
		self.detected_contours = []
		self.points = []
		print ('Start')
		#Subscriber to the camera depth
		self.image_sub_depth = rospy.Subscriber("/camera/depth/points",PointCloud2,self.callback_depth, queue_size = 10)
		print ('depth')
		#print (len(self.points))
		#Subscriber to the camera flow
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback_RGB, queue_size = 10)
		# Publisher of object coordinates
		self.image_pub = rospy.Publisher("recognized_objects", RecognizedObjects,queue_size=10)
		print ('Published')

	def callback_RGB(self,data):  # Callback function RGB data
		# Read the frame and convert it using bridge
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		#cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Color image to grayscale	
		cv_image_temp = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		#cv_image_temp[np.all(cv_image_temp == 255, axis=2)] = 0
		cv_image_temp = cv2.GaussianBlur(cv_image_temp, (5, 5), cv2.BORDER_DEFAULT)
		# Binarization
		#_,cv_image_temp = cv2.threshold(cv_image_temp, 50, 255, cv2.THRESH_BINARY_INV)
		#cv_image_temp = cv2.adaptiveThreshold(cv_image_temp,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
		ret,cv_image_temp = cv2.threshold(cv_image_temp, 20, 255, cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
		#Contours finding
		#cv_image_temp = cv2.Canny(cv_image_temp, 10, 250)
		#kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
		#cv_image_temp = cv2.morphologyEx(cv_image_temp, cv2.MORPH_CLOSE, kernel)		
		#cv_image_temp = cv2.GaussianBlur(cv_image_temp, (3, 3), 0)
		#self.detected_contours 
		
		self.detected_contours = cv2.findContours(cv_image_temp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
		#for c in self.detected_contours:
			#print(c[0])
			#print('1')
    
    		#Count up coordinates
    		coor = []
		width = cv_image_temp.shape[1]
		hight = cv_image_temp.shape[0]

		
		#print(width)
		for c in self.detected_contours:
      			x_mean = .0
 			y_mean = .0
 			z_mean = .0
			for i in range (len(c)):
				index = c[i][0][1]*width + c[i][0][0]
				
				#index = c[i][0][1] + c[i][0][0]*hight
				#print(self.points[index])
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
				if (c[i][0][0] < border) or (c[i][0][0] > width-border) or (c[i][0][1] < border) or (c[i][0][1] > hight-border):
					flag = True
			if (M["m00"] != 0) and (area > 100) and (area < 50000) and not flag:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				#cv2.circle(img1, (cX, cY), 7, (255, 255, 255), -1)
				index = cY*width + cX
				cv2.drawContours(img1, [c], -1, (0,255,0), 3)
				objects_coor.append(self.points[index])
		cv2.imshow("Camera image", img1)
		cv2.waitKey(5)
		pub_msg = RecognizedObjects()
		pub_msg.number_of_objects = len(objects_coor)
		pub_msg.points_data = objects_coor
		self.image_pub.publish(pub_msg)



	def callback_depth(self,msg):
		#print(msg.data)
		# construct a numpy record type equivalent to the point type of this cloud
		data_type_list = extract_dtype_list(msg)
		#print (msg.fields)
		# parse the cloud into an array
		cloud_arr = np.fromstring(msg.data, data_type_list)
		#points_list = []

		#for data in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
			#points_list.append([data[0], data[1], data[2]])
		#gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
		#cloud_arr = np.frombuffer(msg.data, data_type_list)
		#print (cloud_arr[0])
		cloud_arr = cloud_arr[[fname for fname, _type in data_type_list if not (fname[:len(PREFIX)] == PREFIX)]]
		dtype = find_XYZ_datatype(msg)
		cloud_arr = np.reshape(cloud_arr, (msg.height, msg.width))
		self.points = get_points(cloud_arr, dtype=dtype)
		#imm = np.zeros(cloud_arr.shape, dtype)
		#imm = cloud_arr['rgb']
		#print(imm)
		#print (len(self.points))


#--------------- MAIN LOOP
def main(args):
	#--- Create the object
	print('Main')
	loop = main_loop()

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
