# coding=utf-8
# =============================================================================
#  Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
#
#  This software is the confidential and proprietary information of FLIR
#  Integrated Imaging Solutions, Inc. ("Confidential Information"). You
#  shall not disclose such Confidential Information and shall use it only in
#  accordance with the terms of the license agreement you entered into
#  with FLIR Integrated Imaging Solutions, Inc. (FLIR).
#
#  FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
#  SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#  PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
#  SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
#  THIS SOFTWARE OR ITS DERIVATIVES.
# =============================================================================
#
# Acquisition.py shows how to acquire images. It relies on
# information provided in the Enumeration example. Also, check out the
# ExceptionHandling and NodeMapInfo examples if you haven't already.
# ExceptionHandling shows the handling of standard and Spinnaker exceptions
# while NodeMapInfo explores retrieving information from various node types.
#
# This example touches on the preparation and cleanup of a camera just before
# and just after the acquisition of images. Image retrieval and conversion,
# grabbing image data, and saving images are all covered as well.
#
# Once comfortable with Acquisition, we suggest checking out
# AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi.
# AcquisitionMultipleCamera demonstrates simultaneously acquiring images from
# a number of cameras, NodeMapCallback serves as a good introduction to
# programming with callbacks and events, and SaveToAvi exhibits video creation.

import PySpin
import numpy as np
import cv2.aruco as aruco
import cv2
from tf.transformations import *
#from CameraParams import *


class Camera_Class:
	
	def CameraService(self):
		"""
		Example entry point; please see Enumeration example for more in-depth
		comments on preparing and cleaning up the system.

		:return: True if successful, False otherwise.
		:rtype: bool
		"""
		# Retrieve singleton reference to system object
		system = PySpin.System.GetInstance()

		# Retrieve list of cameras from the system
		cam_list = system.GetCameras()

		num_cameras = cam_list.GetSize()

		print "Number of cameras detected: %d" % num_cameras


		# Finish if there are no cameras
		if num_cameras == 0:

			# Clear camera list before releasing system
			cam_list.Clear()

			# Release system
			system.ReleaseInstance()

			print "Not enough cameras!"
			raw_input("Done! Press Enter to exit...")
			return False

		# Run example on each camera
		camera_yaml_files=['camera1_params.yaml']
		camera_params=[]
		for i in range(num_cameras):
			cam = cam_list.GetByIndex(i)
			print cam.DeviceSerialNumber.GetValue()
			with open(camera_yaml_files[i],'r') as stream:

				camera_params[i]=yaml.load(stream)

			#Cam1 = CameraParams(5813.48508684566, 2560.092268747669, 5857.787526106612, 1928.738413521021, 1.0, -0.0862, 0.3082, 0.0, 0.0, 0.0)

			print "Running example for camera %d..." % i

			result = self.run_single_camera(cam)
			print "Camera %d example complete..." % i

		    # Operations on the frame
			if result != False:
				frame = np.array(result.GetData(), dtype="uint8").reshape( (result.GetHeight(), result.GetWidth(),1))
		
				aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
				parameters =  aruco.DetectorParameters_create()
				corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		
				print corners
				idx_All = np.zeros([4,1])
				Corners_All = np.zeros([4,4,2])
				if len(ids)==4:
					for k in range(4):
						m = corners[k]
						idx = ids[k]
						idx_All[idx-1,:] = idx
						idx_All[idx-1,:] = idx
						Corners_All[idx-1,:,:] = m[0,:,:]
					
					Pca, Rca = self.get_object_pose_m(np.reshape(Corners_All[0:4,:,:],[16,2]),camera_params[i]['camMatrix'], camera_params[i]['distCoeff'])
					Rcatmp = np.vstack((np.hstack((Rca,[[0],[0],[0]])),[0,0,0,1]))  
					qca = quaternion_from_matrix(Rcatmp)
					Rca = quaternion_matrix([1.0*qca[1],1.0*qca[0],-1.0*qca[3],-1.0*qca[2]])
					Rca = Rca[0:3,0:3]
					Poa, Roa = self.map_camera2robot(Pca,Rca,camera_params[i]['M'],camera_params[i]['R'])
					Psetpre=np.asarray(self.ARtagparams['Pset'])
					Rset=self.ARtagparams['Rset']

					Pset = np.matmul(Roa,Psetpre) #(-z,x,y)[-0.3,0.538,0.18415]
		            #Rset = [[0.975004629674004,-0.012471665796434,-0.221834239166364],[0.006172005904842,0.999558362226998,-0.029068657437025],[ 0.222098803367709,0.026972913345476,0.974651005995628]] #ZYX
		            #sets the panel position relative to the tag
		            #Rset = [[0.993265901528871,  -0.009385243115853,  -0.115476257610362],[0.009746998214369,   0.999949198262679,   0.002568446930442],[ 0.115446285716984,  -0.003676697632625,   0.993306919843344]] #ZYX
		            #Rset = [[0.993313684906749,  -0.000007686718905 , -0.115446625407115],[0.000280162683008 ,  0.999997213667025 ,  0.002343963962486],[ 0.115446285716984,  -0.002360635317184,   0.993310919357606]] #ZYX
	   
	   
	   
					Roa = np.matmul(Roa,Rset)


					Rtmp = np.vstack((np.hstack((Roa,[[0],[0],[0]])),[0,0,0,1]))
					qoa = quaternion_from_matrix(Rtmp)

					#tmp = euler_from_matrix(Rtmp)
					#                print tmp
					#                qoa2 = quaternion_from_euler(1.0*tmp[2],1.0*tmp[0],-1.0*tmp[0])
					#Roa2 = quaternion_matrix([qoa[3],qoa[0],qoa[1],qoa[2]])
					#Roa2 = Roa2[0:3,0:3]

					print 'Poa:',Poa
					print 'Roa:',Roa
					#print 'Roa2:',Roa2 
					print 'qoa_***:',[qoa[3],qoa[0],qoa[1],qoa[2]]

					print 'set ponit',Pset
					Poa = Poa+Pset


		            #print 'qoa2:',[qoa2[3],qoa2[0],qoa2[1],qoa2[2]]

		            


		# Release reference to camera
		# NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
		# cleaned up when going out of scope.
		# The usage of del is preferred to assigning the variable to None.
		del cam

		# Clear camera list before releasing system
		cam_list.Clear()

		# Release instance
		system.ReleaseInstance()

		#raw_input("Done! Press Enter to exit...")
		return Poa,[qoa[3],qoa[0],qoa[1],qoa[2]]

	def acquire_images(self,cam, nodemap, nodemap_tldevice):
		"""
		:param cam: Camera to acquire images from.
		:param nodemap: Device nodemap.
		:param nodemap_tldevice: Transport layer device nodemap.
		:type cam: CameraPtr
		:type nodemap: INodeMap
		:type nodemap_tldevice: INodeMap
		:return: True if successful, False otherwise.
		:rtype: bool
		"""
		NUM_IMAGES = 1  # number of images to grab
		print "*** IMAGE ACQUISITION ***\n"
		try:
			result = True
			
			# Set acquisition mode to continuous
			#
			#  *** NOTES ***
			#  Because the example acquires and saves 10 images, setting acquisition
			#  mode to continuous lets the example finish. If set to single frame
			#  or multiframe (at a lower number of images), the example would just
			#  hang. This would happen because the example has been written to
			#  acquire 10 images while the camera would have been programmed to
			#  retrieve less than that.
			#
			#  Setting the value of an enumeration node is slightly more complicated
			#  than other node types. Two nodes must be retrieved: first, the
			#  enumeration node is retrieved from the nodemap; and second, the entry
			#  node is retrieved from the enumeration node. The integer value of the
			#  entry node is then set as the new value of the enumeration node.
			#
			#  Notice that both the enumeration and the entry nodes are checked for
			#  availability and readability/writability. Enumeration nodes are
			#  generally readable and writable whereas their entry nodes are only
			#  ever readable.
			#
			#  Retrieve enumeration node from nodemap
			
			# In order to access the node entries, they have to be casted to a pointer type (CEnumerationPtr here)
			node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
			if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
				print "Unable to set acquisition mode to continuous (enum retrieval). Aborting..."
				return False
			
		    # Retrieve entry node from enumeration node
			node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName("Continuous")
			if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
				print "Unable to set acquisition mode to continuous (entry retrieval). Aborting..."
				return False
			
		    # Retrieve integer value from entry node
			acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
			
			# Set integer value from entry node as new value of enumeration node
			node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
			
			print "Acquisition mode set to continuous..."
			
			#  Begin acquiring images
			#
			#  *** NOTES ***
			#  What happens when the camera begins acquiring images depends on the
			#  acquisition mode. Single frame captures only a single image, multi
			#  frame catures a set number of images, and continuous captures a
			#  continuous stream of images. Because the example calls for the
			#  retrieval of 10 images, continuous mode has been set.
			#
			#  *** LATER ***
			#  Image acquisition must be ended when no more images are needed.
			cam.BeginAcquisition()
			
			print "Acquiring images..."
			
			#  Retrieve device serial number for filename
			#
			#  *** NOTES ***
			#  The device serial number is retrieved in order to keep cameras from
			#  overwriting one another. Grabbing image IDs could also accomplish
			#  this.
			device_serial_number = ""
			node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode("DeviceSerialNumber"))
			if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
				device_serial_number = node_device_serial_number.GetValue()
				print "Device serial number retrieved as %s..." % device_serial_number

		    # Retrieve, convert, and save images
			for i in range(NUM_IMAGES):
				try:
					
					#  Retrieve next received image
					#
					#  *** NOTES ***
					#  Capturing an image houses images on the camera buffer. Trying
					#  to capture an image that does not exist will hang the camera.
					#
					#  *** LATER ***
					#  Once an image from the buffer is saved and/or no longer
					#  needed, the image must be released in order to keep the
					#  buffer from filling up.
					image_result = cam.GetNextImage()

					#  Ensure image completion
					#
					#  *** NOTES ***
					#  Images can easily be checked for completion. This should be
					#  done whenever a complete image is expected or required.
					#  Further, check image status for a little more insight into
					#  why an image is incomplete.
					if image_result.IsIncomplete():
						print "Image incomplete with image status %d ..." % image_result.GetImageStatus()

					else:

						#  Print image information; height and width recorded in pixels
						#
						#  *** NOTES ***
						#  Images have quite a bit of available metadata including
						#  things such as CRC, image status, and offset values, to
						#  name a few.
						width = image_result.GetWidth()
						height = image_result.GetHeight()
						print "Grabbed Image %d, width = %d, height = %d" % (i, width, height)

						#  Convert image to mono 8
						#
						#  *** NOTES ***
						#  Images can be converted between pixel formats by using
						#  the appropriate enumeration value. Unlike the original
						#  image, the converted one does not need to be released as
						#  it does not affect the camera buffer.
						#
						#  When converting images, color processing algorithm is an
						#  optional parameter.
						image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)

						# Create a unique filename
						if device_serial_number:
							filename = "Acquisition3-%s-%d.jpg" % (device_serial_number, i)
						else:  # if serial number is empty
							filename = "Acquisition-%d.jpg" % i

						#  Save image
						#
						#  *** NOTES ***
						#  The standard practice of the examples is to use device
						#  serial numbers to keep images of one device from
						#  overwriting those of another.
						image_result.Save(filename)
						print "Image saved at %s" % filename

						#  Release image
						#
						#  *** NOTES ***
						#  Images retrieved directly from the camera (i.e. non-converted
						#  images) need to be released in order to keep from filling the
						#  buffer.
						image_result.Release()
						print ""
						#raw_input("Press Enter to continue...")



				except PySpin.SpinnakerException as ex:
					print "Error: %s" % ex
					return False

		    #  End acquisition
		    #
		    #  *** NOTES ***
		    #  Ending acquisition appropriately helps ensure that devices clean up
		    #  properly and do not need to be power-cycled to maintain integrity.
			cam.EndAcquisition()
		
		except PySpin.SpinnakerException as ex:
			print "Error: %s" % ex
			return False

		return image_converted


	def print_device_info(self,nodemap):
		"""
		This function prints the device information of the camera from the transport
		layer; please see NodeMapInfo example for more in-depth comments on printing
		device information from the nodemap.

		:param nodemap: Transport layer device nodemap.
		:type nodemap: INodeMap
		:returns: True if successful, False otherwise.
		:rtype: bool
		"""

		print "*** DEVICE INFORMATION ***\n"

		try:
			result = True
			node_device_information = PySpin.CCategoryPtr(nodemap.GetNode("DeviceInformation"))

			if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
				features = node_device_information.GetFeatures()
				for feature in features:
					node_feature = PySpin.CValuePtr(feature)
					print "%s: %s" % (node_feature.GetName(),
		                              node_feature.ToString() if PySpin.IsReadable(node_feature) else "Node not readable")

			else:
				print "Device control information not available."

		except PySpin.SpinnakerException as ex:
			print "Error: %s" % ex
			return False

		return result


	def run_single_camera(self,cam):
		"""
		This function acts as the body of the example; please see NodeMapInfo example
		for more in-depth comments on setting up cameras.

		:param cam: Camera to run on.
		:type cam: CameraPtr
		:return: True if successful, False otherwise.
		:rtype: bool
		"""
		try:
			result = True

			# Retrieve TL device nodemap and print device information
			nodemap_tldevice = cam.GetTLDeviceNodeMap()

			result &= self.print_device_info(nodemap_tldevice)

			# Initialize camera
			cam.Init()

			# Retrieve GenICam nodemap
			nodemap = cam.GetNodeMap()

			# Acquire images
			result = self.acquire_images(cam, nodemap, nodemap_tldevice)

			# Deinitialize camera
			cam.DeInit()

		except PySpin.SpinnakerException as ex:
			print "Error: %s" % ex
			result = False

		return result



	def map_camera2robot(self,Pca,Rca,M,R):
		
		
		Poa = np.matmul(np.vstack((Pca,1)).T,M)
		Roa = np.matmul(Rca,R)
		#Roa = np.matmul(Roa,r)
		return [Poa, Roa]


	def get_object_pose_m(self,corners, camMatrix, distCoeff):
		# AR Tag Dimensions
		d = 65.9
		with open("ARtag1.yaml",'r') as stream:
			self.ARtagparams=yaml.load(stream)
		
		objPoints=np.asarray(ARtagparams['points'])

		# SolvePnP
		retVal, rvec, tvec = cv2.solvePnP(objPoints, corners, camMatrix, distCoeff)
		Rca, b = cv2.Rodrigues(rvec)
		Pca = tvec

		return [Pca, Rca]
    
    


	

#if __name__ == "__main__":
#    main()
