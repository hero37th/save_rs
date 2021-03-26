

#####################################################
##                        F                        ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import time for acquire time now
import time

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
        
        t1 = time.time()
        print(t1)
        color_image = np.asanyarray(color_frame.get_data())
        str1 = './rgb/'+str(t1)+'.png'
        print(str1)
        cv2.imwrite(str1, color_image)
        t2 = time.time()
        print(t2)
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_colormap = cv2.convertScaleAbs(depth_image, alpha=0.03)
        #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) 
        #depth image is 1 channel, color is 3 channels
        str2 = './depth/'+str(t2)+'.png'
        print(str2)
        cv2.imwrite(str2, depth_colormap)
finally:
    pipeline.stop()
