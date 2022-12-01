import pyzed.sl as sl
import cv2

def get_image():

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_image_flip = sl.FLIP_MODE.OFF
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Grab an image
    runtime_parameters = sl.RuntimeParameters()
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns ERROR_CODE.SUCCESS

        image = sl.Mat()

        key = ''
        while key != 113:  # for 'q' key
            err = zed.grab(runtime_parameters)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                cv2.imshow("ZED", image.get_data())
                key = cv2.waitKey(5)
            else:
                key = cv2.waitKey(5)
        cv2.destroyAllWindows()

        # cv2.imwrite('img.png', image.get_data())

    # Close the camera
    zed.close()

    return image

get_image()