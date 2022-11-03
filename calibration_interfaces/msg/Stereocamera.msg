std_msgs/Header header  # timestamp in the header is the time of the reading

sensor_msgs/Image image             # left image
sensor_msgs/CameraInfo camera_info  # left camera info
sensor_msgs/PointCloud2 cloud

sensor_msgs/PointCloud2 centers     # detected circle centers