#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs
import logging

def get_realsense_devices():
  
    devices = rs.context().query_devices()
    device_list = []

    for device in devices:
        device_info = {
            "name": device.get_info(rs.camera_info.name),
            "serial_number": device.get_info(rs.camera_info.serial_number),
            "product_id": device.get_info(rs.camera_info.product_id),
            "product_line": device.get_info(rs.camera_info.product_line),
        }
        device_list.append(device_info)

    if not device_list:
        print("No RealSense devices found.")
    else:
        print("Connected RealSense Devices:")
        for idx, device in enumerate(device_list, start=1):
            print(f"Device {idx}: {device}")

    return device_list



class RealSense_Cam:

    def __init__(self,camera_serial_number = None) -> None:
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.context = rs.context()
        self.serial_number = camera_serial_number
   



    def start_real_sense(self):
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        if self.serial_number is not None:
            config.enable_device(self.serial_number)
        
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            logging.debug("The demo requires Depth camera with Color sensor")
            
            #print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        return pipeline, config
    

    def get_frame_from_realsense(self, pipeline,aligned_frame=True):
        frames = pipeline.wait_for_frames()

        if aligned_frame == True:
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
              # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                return None, None

            depth_image_aligned = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.03), cv2.COLORMAP_JET)
            # Wait for a coherent pair of frames: depth and color
            depth_colormap_aligned_dim = depth_colormap_aligned.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_aligned_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_aligned_dim[1], depth_colormap_aligned_dim[0]),
                                                interpolation=cv2.INTER_AREA)
                images = resized_color_image
            else:
                images = color_image

            return depth_colormap_aligned, images
        else:
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                return None, None

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                interpolation=cv2.INTER_AREA)
                images = resized_color_image
            else:
                images = color_image

            return depth_colormap, images
    
       




    def depth_information(self,pipeline):
        # Get the depth sensor from the RealSense pipeline
        depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()

        # Get the depth scale
        depth_scale = depth_sensor.get_depth_scale()

        # Get the depth sensor intrinsic parameters
        depth_stream = pipeline.get_active_profile().get_stream(rs.stream.depth)
        depth_intrin = depth_stream.as_video_stream_profile().get_intrinsics()

        
        return depth_scale, depth_intrin
    

if __name__ == "__main__":


    list_of_devices = get_realsense_devices()
    for index, device in  enumerate(list_of_devices):
        
        cam = RealSense_Cam(device["serial_number"])
        pipeline, config = cam.start_real_sense()
        # Start streaming
        pipeline.start(config)



    while True:

        depth, frame = cam.get_frame_from_realsense(pipeline,aligned_frame=False)

        
        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', frame)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
    
    pipeline.stop()