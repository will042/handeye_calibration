import pyzed.sl as sl
'''
Used to convert from pixels to x, y, z coordinate in camera frame.
Input: x, y in pixels
Output: x, y, z in mm
'''

def get_3d_point(x, y):

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_image_flip = sl.FLIP_MODE.OFF
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units (for depth measurements)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    point_cloud = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)  # Retrieve 3d point

        point3d = point_cloud.get_value(x, y)[1][0:3]

    zed.close()

    return point3d

print(get_3d_point(1920/2, 1080/2))