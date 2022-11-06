# A single synchronized reading from a pair of sensors and its predicted 
# transformation from the calibration node

std_msgs/Header header  # timestamp in the header is the time the calibration node produce the prediction

# Sensor type enums
uint8 LIDAR=0
uint8 MONOCAMERA=1
uint8 STEREOCAMERA=2

uint8 sensor1_type              # the sensor 1 type (lidar, monocamera, stereocamera) [enum]
uint8 sensor2_type              # the sensor 2 type (lidar, monocamera, stereocamera) [enum]

sensor_msgs/Image image1        # Image reading from sensor 1 (monocamera, stereocamera)
sensor_msgs/Image image2        # Image reading from sensor 2 (monocamera, stereocamera)

sensor_msgs/CameraInfo camera1  # CameraInfo from sensor 1 (monocamera, stereocamera)
sensor_msgs/CameraInfo camera2  # CameraInfo from sensor 2 (monocamera, stereocamera)

sensor_msgs/PointCloud2 cloud1  # PointCloud2 reading from sensor 1 (lidar, stereocamera)
sensor_msgs/PointCloud2 cloud2  # PointCloud2 reading from sensor 2 (lidar, stereocamera)

float64[12]  tf_sensor1_board

float64[12]  tf_sensor2_board

float64[12]  tf_sensor1_sensor2