uint64 timestamp
bool is_detected
float32 x
float32 y
# Raw undistorted camera-pixel coordinates of the same detection, before the
# table homography - x/y above are table-frame mm. Kept separately since
# overlaying on the raw camera image (e.g. plot_detections.py) needs pixel
# coordinates, not table mm.
float32 image_x
float32 image_y
