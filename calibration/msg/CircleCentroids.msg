std_msgs/Header header  # timestamp in the header is the time of the reading

# Sensor type enums
uint8 LIDAR=0
uint8 MONOCAMERA=1
uint8 STEREOCAMERA=2

uint8 sensor_type                   # the sensor type (lidar, monocamera, stereocamera) [enum]

sensor_msgs/Image image
sensor_msgs/CameraInfo camera_info
sensor_msgs/PointCloud2 cloud

sensor_msgs/PointCloud2 centers     # detected circle centers