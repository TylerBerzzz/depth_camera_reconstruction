import pyrealsense2 as rs
import numpy as np
import cv2


try:
    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        # color_frame= frames.get_color_frame()
        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())

        # Normalize the depth image to the range 0-255 for display
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imshow('Depth Stream', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pipeline.stop()

except Exception as e:
    print(e)

finally:
    cv2.destroyAllWindows()
