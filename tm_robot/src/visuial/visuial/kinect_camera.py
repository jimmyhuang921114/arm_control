import cv2
import numpy as np
import sys

import pyk4a
from pyk4a import Config, PyK4A, ColorResolution, DepthMode, ImageFormat, FPS, connected_device_count
from helpers import colorize # Import colorize function from helpers.py

# === Camera Control and Configuration Settings (Modify these variables directly) ===

# Color Image Resolution
# Available options: ColorResolution.OFF, ColorResolution.RES_720P, ColorResolution.RES_1080P,
#                    ColorResolution.RES_1440P, ColorResolution.RES_1536P, ColorResolution.RES_2160P,
#                    ColorResolution.RES_3072P
SET_COLOR_RESOLUTION = ColorResolution.RES_1080P

# Depth Sensor Mode
# Available options: DepthMode.OFF, DepthMode.NFOV_2X2BINNED, DepthMode.NFOV_UNBINNED,
#                    DepthMode.WFOV_2X2BINNED, DepthMode.WFOV_UNBINNED, DepthMode.PASSIVE_IR
# NFOV_UNBINNED or WFOV_UNBINNED are generally recommended for better transformation results
SET_DEPTH_MODE = DepthMode.NFOV_UNBINNED

# Camera Frame Rate (FPS)
# Available options: FPS.FPS_5, FPS.FPS_15, FPS.FPS_30
# Note: Some resolution/mode combinations may not support all frame rates. Refer to Azure Kinect DK documentation.
SET_CAMERA_FPS = FPS.FPS_30

# Color Image Format
# Available options: ImageFormat.COLOR_BGRA32, ImageFormat.COLOR_MJPG,
#                    ImageFormat.COLOR_NV12, ImageFormat.COLOR_YUY2
# BGRA32 is typically uncompressed for best quality but larger data size; MJPG is compressed, smaller data size.
SET_COLOR_FORMAT = ImageFormat.COLOR_BGRA32

# Synchronized Images Only
# Set to True to ensure each capture contains synchronized color and depth images, necessary for transformation.
# Set to False to get unsynchronized streams, potentially useful for highest frame rates (sacrificing sync).
SET_SYNCHRONIZED_IMAGES_ONLY = True

# === Image Display Settings ===
# Depth range for colorizing the depth image (in millimeters)
# Set to (None, None) to use the min/max values from the image.
# Set to (min, max) to map depth values within the specified range to colors.
SET_DEPTH_COLORIZATION_RANGE = (None, None) # Example: display depth from 0 to 5 meters

# === Main Application Logic ===

def main():
    # --- Device Selection ---
    num_connected_devices = connected_device_count() # Get the number of connected devices
    if num_connected_devices == 0:
        print("No Azure Kinect DK devices detected. Please connect a device and try again.")
        sys.exit(1) # Exit if no devices are found

    print(f"Detected {num_connected_devices} Azure Kinect DK device(s):")

    devices = {}
    for device_id in range(num_connected_devices):
        try:
            # Temporarily open the device to get its serial number
            device = PyK4A(device_id=device_id)
            device.open()
            serial_number = device.serial
            device.close()
            devices[device_id] = serial_number
            print(f"  ID: {device_id}, Serial Number: {serial_number}")
        except Exception as e:
             print(f"  Could not get info for device ID {device_id}: {e}")
             # Record devices even if serial number can't be retrieved
             devices[device_id] = "Could not retrieve serial number"


    if not devices:
         print("Could not retrieve valid information for any device. Please check device connections and drivers.")
         sys.exit(1)


    selected_device_id = None
    if num_connected_devices > 1:
        while selected_device_id is None:
            try:
                choice = int(input(f"Enter the ID of the device to use (0 to {num_connected_devices - 1}): "))
                if choice in devices:
                    selected_device_id = choice
                else:
                    print("Invalid device ID. Please enter again.")
            except ValueError:
                print("Invalid input. Please enter a number.")
    else:
        selected_device_id = 0 # Automatically select ID 0 if only one device is found
        print(f"Automatically selected device ID {selected_device_id}.")
    # --- End Device Selection ---


    # Create the configuration object based on the settings at the top
    config = Config(
        color_resolution=SET_COLOR_RESOLUTION,
        depth_mode=SET_DEPTH_MODE,
        camera_fps=SET_CAMERA_FPS,
        color_format=SET_COLOR_FORMAT,
        synchronized_images_only=SET_SYNCHRONIZED_IMAGES_ONLY,
    )

    # Create and start the Azure Kinect device using the selected device ID
    try:
        k4a = PyK4A(config, device_id=selected_device_id) # Use the selected device_id
        k4a.start()
        print(f"\nSuccessfully started device ID {selected_device_id} ({devices.get(selected_device_id, 'Unknown Device')}) with the following configuration:")
        print(f"  Color Resolution: {SET_COLOR_RESOLUTION.name}")
        print(f"  Depth Mode: {SET_DEPTH_MODE.name}")
        print(f"  Camera FPS: {SET_CAMERA_FPS.name}")
        print(f"  Color Format: {SET_COLOR_FORMAT.name}")
        print(f"  Synchronized Images: {SET_SYNCHRONIZED_IMAGES_ONLY}")
        print("\nPress 'q' to exit.")

    except Exception as e:
        print(f"Failed to start device ID {selected_device_id}: {e}")
        print("Please check:")
        print("- If the device is properly connected.")
        print("- If the Azure Kinect SDK is correctly installed.")
        print("- If the selected resolution, mode, FPS, and format combination is supported by the hardware.")
        sys.exit(1) # Exit the program if startup fails

    while True:
        # Get a capture
        # If synchronized_images_only=True and both color/depth are enabled,
        # the capture object will include color, depth, ir, transformed_depth, transformed_color, transformed_ir
        # Otherwise, it will only contain the raw images for the configured streams.
        capture = k4a.get_capture()

        # Display Color Image (if available)
        if capture.color is not None:
            # Convert the color image from its original format to a displayable format for OpenCV (BGRA or BGR)
            color_image_display = None
            try:
                if SET_COLOR_FORMAT == ImageFormat.COLOR_BGRA32:
                     color_image_display = capture.color[:, :, :3] # Take BGR channels
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_MJPG:
                      # MJPG format is usually decoded by pyk4a, use directly
                      color_image_display = capture.color
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_NV12:
                      # Convert NV12 to BGRA then take BGR
                      color_image_display = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_NV12)[:, :, :3]
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_YUY2:
                      # Convert YUY2 to BGRA then take BGR
                      color_image_display = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_YUY2)[:, :, :3]
                else:
                    # For other unknown formats, try to display directly or ignore
                    color_image_display = capture.color[:, :, :3] # Default assumption similar to BGRA
                    print(f"Warning: Using default display handling for color format {SET_COLOR_FORMAT.name}", flush=True)

            except Exception as e:
                 print(f"Warning: Failed to convert color image format {SET_COLOR_FORMAT.name} for display: {e}", flush=True)
                 color_image_display = None # Do not display color image if conversion fails

            if color_image_display is not None:
                 cv2.imshow("Color Image", color_image_display)


        # Display Transformed Depth Image (if available and configured for synchronized capture)
        # capture.transformed_depth is available when synchronized_images_only=True and both color/depth are enabled
        if SET_SYNCHRONIZED_IMAGES_ONLY and capture.transformed_depth is not None:
             try:
                # Colorize the transformed depth image
                depth_colorized = colorize(capture.transformed_depth, SET_DEPTH_COLORIZATION_RANGE, cv2.COLORMAP_HSV)
                cv2.imshow("Transformed Depth Image", depth_colorized)
             except Exception as e:
                 print(f"Warning: Error processing transformed depth image: {e}", flush=True)

        # Display Original Depth Image (if available and not showing transformed, or if color is off)
        # This handles cases where synchronized_images_only=False or color_resolution=OFF
        # Note: If synchronized_images_only=True and color is OFF, original depth might also be None
        elif capture.depth is not None:
             try:
                 original_depth_colorized = colorize(capture.depth, SET_DEPTH_COLORIZATION_RANGE, cv2.COLORMAP_HSV)
                 cv2.imshow("Original Depth Image", original_depth_colorized)
             except Exception as e:
                 print(f"Warning: Error processing original depth image: {e}", flush=True)


        # Check for key press to exit
        key = cv2.waitKey(1)  # Wait 1ms for a key press
        if key == ord('q'):  # Press 'q' to quit
            break

    # Clean up resources
    k4a.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()