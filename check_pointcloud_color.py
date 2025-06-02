import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

# Enable depth stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    pipeline_profile = pipeline.start(config)
    print("Pipeline started successfully.")
except RuntimeError as e:
    print(f"Failed to start pipeline: {e}")
    exit(1)

try:
    while True:
        try:
            # Wait for frames with a longer timeout
            frames = pipeline.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                print("No depth frame received.")
                continue

            print("Depth frame received successfully.")
            break  # Exit after the first successful frame

        except RuntimeError as e:
            print(f"Error while waiting for frames: {e}")
            continue

finally:
    pipeline.stop()