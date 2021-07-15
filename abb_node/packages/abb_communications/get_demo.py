import abb
import pyrealsense2 as rs
import cv2
import numpy as np
import time


# Robot macros
img_h = 640
img_w = 480
PIX_SIZE_U = 516.0/float(img_h) # mm
PIX_SIZE_V =  390.0/float(img_w) # mm
TOOL_RS = [[31.99789,42.31533,121.1225],[0.68301,-0.18301,0.68301,0.18301]] #Realsense tool, from tool0
TOOL_GRIP = [[-11.0,0.71,278.8],[0.965925826,0,0,0.258819045]]
TABLE_H = 500 # h offset in millimeters
out_filename = "demo_fold_abb.txt"

def xyz_from_pix(u, v, pix_size_u, pix_size_v, off_u=320, off_v=240, height=500):
    x = ((v - off_v) * pix_size_v)
    y = ((u - off_u) * pix_size_u)
    z = height
    return [x, y, z]


# Initialize connection to abb
robot = abb.Robot()
robot.set_units('millimeters', 'radians')

# Initialize realsense camera.
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
# Optimal res for depth images is 848x480, see: https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf
config.enable_stream(rs.stream.depth, img_h, img_w, rs.format.z16, 30) #TODO
config.enable_stream(rs.stream.color, img_h, img_w, rs.format.bgr8, 30)
pipeline.start(config)

i = 0
done = True

while done:

    # Move robot to camera ready position
    robot.set_tool(TOOL_RS)
    robot.set_cartesian([[0,0,0],[1,0,0,0]]) # cart: x,y,z. quat: w,x,y,z
    time.sleep(5)

    # Get obs from camera pipelline
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue  # Convert images to numpy arrays  depth = np.asanyarray(depth_frame.get_data())  color = np.asanyarray(color_frame.get_data())

    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

    # Convert images to numpy arrays
    depth = np.asanyarray(depth_frame.get_data())
    color = np.asanyarray(color_frame.get_data())

    # Visualize images
    cv2.imwrite('color_img_'+ str(i) + '.jpg', color)
    cv2.imwrite('depth_img_'+ str(i) + '.jpg', depth)

    # Picture path
    img = color
    u = []
    v = []

    def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            a.append(x)
            b.append(y)
            cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
            cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                        1.0, (0, 0, 0), thickness=1)
            cv2.imshow("image", img)
            print(x,y)

    cv2.namedWindow("image")
    cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    cv2.imshow("image", img)
    cv2.waitKey(0)
    
    # transform first pix selected to position pick
    APROX_H = -100.0
    pick_aprox_pos = xyz_from_pix(v[0], u[0], PIX_SIZE_V, PIX_SIZE_U, height=TABLE_H+APROX_H)
    pick_pos = xyz_from_pix(v[0], u[0], PIX_SIZE_V, PIX_SIZE_U)
    place_aprox_pos = xyz_from_pix(v[0], u[0], PIX_SIZE_V, PIX_SIZE_U, height=TABLE_H+APROX_H)
    place_pos = xyz_from_pix(v[0], u[0], PIX_SIZE_V, PIX_SIZE_U)

    print("Pixels selected for picking: ", u[0], v[0])
    print("Pixels selected for placing: ", u[1], v[1])
    print("Pixel transf for picking corresponds to x,y,z: ", pick_pos)
    print("Pixel transf for placing corresponds to x,y,z: ", place_pos)

    robot.set_tool(TOOL_GRIP)
    #robot.pick_and_place(pick_pos[0], pick_pos[1],
    #place_pos[0], place_pos[1], TABLE_H)
    robot.set_cartesian([pick_aprox_pos, [1,0,0,0]])
    robot.set_cartesian([pick_pos, [1,0,0,0]])
    robot.set_cartesian([pick_aprox_pos, [1,0,0,0]])

    robot.set_cartesian([place_aprox_pos, [1,0,0,0]])
    robot.set_cartesian([place_pos, [1,0,0,0]])
    robot.set_cartesian([place_aprox_pos, [1,0,0,0]])

    with open(out_filename, 'a') as f:
        f.write(str(i) +","+ str(u[0]) +","+ str(v[0]) +","+ str(u[1]) +","+ str(v[1]) +","+ str(pick_pos) +","+ str(place_pos))

    i+=1
    
