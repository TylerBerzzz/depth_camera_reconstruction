import pyrealsense2 as rs
import numpy as np
import time
import os
import cv2
import time


class DepthCamera:
    def __init__(self, resolution_width, resolution_height):
        # Configure depth and color streams

        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        # Get depth scale of the device
        self.depth_scale =  depth_sensor.get_depth_scale()
        # Create an align object
        align_to = rs.stream.color

        self.align = rs.align(align_to)
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print("device product line:", device_product_line)
        config.enable_stream(rs.stream.depth,  resolution_width,  resolution_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)

        
        # Start streaming
        self.pipeline.start(config)
        

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        ir_frame = frames.get_infrared_frame()

        if not depth_frame or not ir_frame:
            return False, None, None
        return True, depth_frame, ir_frame
    
    def get_depth_scale(self):
        """
        "scaling factor" refers to the relation between depth map units and meters; 
        it has nothing to do with the focal length of the camera.
        Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
        """
        return self.depth_scale

    def release(self):
        self.pipeline.stop()       


def depth2PointCloud(depth, rgb, depth_scale, clip_distance_max):
    
    intrinsics = depth.profile.as_video_stream_profile().intrinsics
    depth = np.asanyarray(depth.get_data()) * depth_scale # 1000 mm => 0.001 meters
    rgb = np.asanyarray(rgb.get_data())
    rows,cols  = depth.shape

    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)

    valid = (depth > 0) & (depth < clip_distance_max) #remove from the depth image all values above a given value (meters).
    valid = np.ravel(valid)
    z = depth 
    x =  z * (c - intrinsics.ppx) / intrinsics.fx
    y =  z * (r - intrinsics.ppy) / intrinsics.fy
   
    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]
    
    ir_rgb = cv2.cvtColor(rgb, cv2.COLOR_GRAY2RGB)

    r = np.ravel(ir_rgb[:,:,0])[valid]
    g = np.ravel(ir_rgb[:,:,1])[valid]
    b = np.ravel(ir_rgb[:,:,2])[valid]

    
    pointsxyzrgb = np.dstack((x, y, z, r, g, b))
    pointsxyzrgb = pointsxyzrgb.reshape(-1,6)

    return pointsxyzrgb


def create_point_cloud_file2(vertices, filename):
    ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
    with open(filename, 'w') as f:
        f.write(ply_header %dict(vert_num=len(vertices)))
        np.savetxt(f,vertices,'%f %f %f %d %d %d')


def retrieve_pc():
    ret , depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()

    if not ret:
        print("Unable to get a frame")
    
    points_xyz_rgb = depth2PointCloud(depth_raw_frame, color_raw_frame, depth_scale, clip_distance_max)
    color_frame = np.asanyarray(color_raw_frame.get_data())
    cv2.imshow("Frame",  color_frame )
    
    return points_xyz_rgb


# Directory to save point cloud files
save_directory = r'C:\data\pointclouds'
os.makedirs(save_directory, exist_ok=True)

resolution_width, resolution_height = (640, 480)
clip_distance_max = 1.000 ##remove from the depth image all values above a given value (meters).
Realsensed435Cam = DepthCamera(resolution_width, resolution_height)
depth_scale = Realsensed435Cam.get_depth_scale()


capture_count = 0
while True:
    
    # Save the point cloud in numpy format every 5 seconds
    if capture_count == 0 or time.time() - capture_time >= 5:
        vertices = retrieve_pc()
        capture_time = time.time()
        filename = os.path.join(save_directory, f"pointcloud_{capture_count}.ply")

        create_point_cloud_file2(vertices, filename)
        print(f"Saved point cloud {capture_count} to {filename}")
        capture_count += 1

    key = cv2.waitKey(1)
    if key == 27:  # Press 'ESC' to exit
        Realsensed435Cam.release()
        exit()

