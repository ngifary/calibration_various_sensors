/*
  laser2cam_utils: Helper functions
*/

#ifndef laser2cam_utils_H
#define laser2cam_utils_H

#define PCL_NO_PRECOMPILE
#define DEBUG 1

#include "pcl_ros/transforms.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/distances.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/impl/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#define TARGET_NUM_CIRCLES 4
#define GEOMETRY_TOLERANCE 0.06

namespace LaserScanner
{
  struct Point
  {
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
  } EIGEN_ALIGN16;

  struct PointSphere
  {
    float radius; // linear
    float theta;  // polar
    float phi;    // azimuth
  };

  void addRange(pcl::PointCloud<LaserScanner::Point> &pc)
  {
    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
         pt < pc.points.end(); pt++)
    {
      pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    }
  }

  /**
   * @brief Returns the magnitude of the vector.
   *
   * @param p
   * @return float
   */
  float magnitude(pcl::PointXYZ &p)
  {
    return (std::sqrt(p.x * p.x + p.y + p.y + p.z * p.z));
  }

  /**
   * @brief Converts a cartesian coordinate (x, y, z) into a spherical one (radius, theta, phi).
   *
   * @param pcart
   * @return PointSphere
   */
  PointSphere toSpherical(pcl::PointXYZ &pcart)
  {
    PointSphere psphere;
    psphere.radius = magnitude(pcart);
    psphere.theta = std::atan2(std::sqrt(pcart.x * pcart.x + pcart.y * pcart.y), pcart.z); // -pi/2 <= theta <= pi/2
    psphere.phi = std::atan2(pcart.y, pcart.x);                                            // -pi <= phi <= pi
    return psphere;
  }

  /**
   * @brief Converts a spherical coordinate (radius, theta, phi) into a cartesian one (x, y, z).
   *
   * @param psphere
   * @return pcl::PointXYZ
   */
  pcl::PointXYZ toCartesian(PointSphere &psphere)
  {
    pcl::PointXYZ pcart;
    pcart.x = psphere.radius * std::sin(psphere.theta) * std::cos(psphere.phi);
    pcart.y = psphere.radius * std::sin(psphere.theta) * std::sin(psphere.phi);
    pcart.z = psphere.radius * std::cos(psphere.theta);
    return pcart;
  }

  /**
   * @brief Project the points in the cloud onto the plane
   *
   * @param pc
   * @param model
   */
  void toPlane(pcl::PointCloud<pcl::PointXYZ> &pc, pcl::ModelCoefficients &model)
  {
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      PointSphere psphere = toSpherical(*pt);
      float a, b, c, d;
      a = model.values[0];
      b = model.values[1];
      c = model.values[2];
      d = model.values[3];
      psphere.radius = -(d) / (a * std::sin(psphere.theta) * std::cos(psphere.phi) + b * std::sin(psphere.theta) * std::sin(psphere.phi) + c * std::cos(psphere.theta));
      *pt = toCartesian(psphere);
    }
    return;
  }

  // all intensities to range min-max
  void normalizeIntensity(pcl::PointCloud<Point> &pc, float minv, float maxv)
  {
    float min_found = INFINITY;
    float max_found = -INFINITY;

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
         pt < pc.points.end(); pt++)
    {
      max_found = std::max(max_found, pt->intensity);
      min_found = std::min(min_found, pt->intensity);
    }

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
         pt < pc.points.end(); pt++)
    {
      pt->intensity =
          (pt->intensity - min_found) / (max_found - min_found) * (maxv - minv) +
          minv;
    }
  }
} // namespace LaserScanner

POINT_CLOUD_REGISTER_POINT_STRUCT(LaserScanner::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, range, range))

/**
 * @brief Convert camera to lidar convention or right-down-forward to forward-left-up setEuler(-90,0,-90)
 *
 * @param camera
 * @param lidar
 */
void camera_to_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr camera, pcl::PointCloud<pcl::PointXYZ>::Ptr lidar)
{
  Eigen::Vector3f translation(0.0, 0.0, 0.0);

  float roll = 0.0, pitch = 0.0, yaw = 0.0;

  Eigen::Quaternionf quaternion;

  roll = -M_PI_2;
  pitch = M_PI_2;
  yaw = 0.0;

  quaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  pcl::transformPointCloud(*camera, *lidar, translation, quaternion);
}

/**
 * @brief Convert lidar to camera convention or forward-left-up to right-down-forward setEuler(90,0,90)
 *
 * @param lidar
 * @param camera
 */
void lidar_to_camera(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar, pcl::PointCloud<pcl::PointXYZ>::Ptr camera)
{
  Eigen::Vector3f translation(0.0, 0.0, 0.0);

  float roll = 0.0, pitch = 0.0, yaw = 0.0;

  Eigen::Quaternionf quaternion;

  roll = M_PI_2;
  pitch = 0.0;
  yaw = M_PI_2;

  quaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  pcl::transformPointCloud(*lidar, *camera, translation, quaternion);
}

pcl::PointCloud<pcl::PointXYZ> sortPatternCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
  // 0 -- 1
  // |    |
  // 3 -- 2

  pcl::PointCloud<pcl::PointXYZ> pc_out;
  pc_out.resize(pc->size());

  // Transform points to polar coordinates
  ushort top_pt = 0;
  std::vector<LaserScanner::PointSphere> sphere_centers(4);
  for (ushort i = 0; i < (ushort)pc->size(); i++)
  {
    LaserScanner::PointSphere sphere_center = LaserScanner::toSpherical(pc->at(i));
    sphere_centers[i] = sphere_center;

    if (sphere_centers.at(i).theta < sphere_centers.at(top_pt).theta)
    {
      top_pt = i;
    }
  }

  ushort farthest = 3 - top_pt; // anything but the top_pt
  ushort closest = farthest;

  for (ushort i = 0; i < (ushort)pc->size(); i++)
  {
    if (i != top_pt)
    {
      if (pcl::squaredEuclideanDistance(pc->at(i), pc->at(top_pt)) >
          pcl::squaredEuclideanDistance(pc->at(farthest), pc->at(top_pt)))
      {
        if (pcl::squaredEuclideanDistance(pc->at(farthest), pc->at(top_pt)) <
            pcl::squaredEuclideanDistance(pc->at(closest), pc->at(top_pt)))
        {
          closest = farthest;
        }
        farthest = i;
      }
      else
      {
        if (pcl::squaredEuclideanDistance(pc->at(i), pc->at(top_pt)) <
            pcl::squaredEuclideanDistance(pc->at(closest), pc->at(top_pt)))
        {
          closest = i;
        }
      }
    }
  }
  ushort top_pt2 = 6 - (top_pt + farthest + closest); // 0 + 1 + 2 + 3 = 6
  // diagonally opposing pairs are top_pt-farthest and top_pt2-closest

  ushort lefttop_pt, righttop_pt, leftbottom_pt, rightbottom_pt;

  if (sphere_centers[top_pt].phi > sphere_centers[top_pt2].phi)
  {
    lefttop_pt = top_pt;
    righttop_pt = top_pt2;
    leftbottom_pt = closest;
    rightbottom_pt = farthest;
  }
  else
  {
    lefttop_pt = top_pt2;
    righttop_pt = top_pt;
    leftbottom_pt = farthest;
    rightbottom_pt = closest;
  }

  // Swap indices if target is located in the pi,-pi discontinuity
  double angle_diff = sphere_centers[lefttop_pt].phi - sphere_centers[righttop_pt].phi;
  if (angle_diff > M_PI - sphere_centers[lefttop_pt].phi)
  {
    ushort aux = lefttop_pt;
    lefttop_pt = righttop_pt;
    righttop_pt = aux;

    aux = leftbottom_pt;
    leftbottom_pt = rightbottom_pt;
    rightbottom_pt = aux;
  }

  // Fill vector with sorted centers
  pc_out[0] = pc->points[lefttop_pt];     // lt
  pc_out[1] = pc->points[righttop_pt];    // rt
  pc_out[2] = pc->points[rightbottom_pt]; // rb
  pc_out[3] = pc->points[leftbottom_pt];  // lb
  return pc_out;
}

void colourCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr pc_coloured)
{
  float intensity = 0;
  for (int i = 0; i < 4; i++)
  {
    pcl::PointXYZI cc;

    cc.x = pc->at(i).x;
    cc.y = pc->at(i).y;
    cc.z = pc->at(i).z;
    cc.intensity = intensity;

    pc_coloured->push_back(cc);
    intensity += 0.3;
  }
}

void getCenterClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud,
                       double cluster_tolerance = 0.10,
                       int min_cluster_size = 15, int max_cluster_size = 200,
                       bool verbosity = true)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(cluster_tolerance);
  euclidean_cluster.setMinClusterSize(min_cluster_size);
  euclidean_cluster.setMaxClusterSize(max_cluster_size);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud_in);
  euclidean_cluster.extract(cluster_indices);

  if (DEBUG && verbosity)
    std::cout << cluster_indices.size() << " clusters found from "
              << cloud_in->points.size() << " points in cloud" << std::endl;

  for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();
       it < cluster_indices.end(); it++)
  {
    float accx = 0., accy = 0., accz = 0.;
    for (std::vector<int>::iterator it2 = it->indices.begin();
         it2 < it->indices.end(); it2++)
    {
      accx += cloud_in->at(*it2).x;
      accy += cloud_in->at(*it2).y;
      accz += cloud_in->at(*it2).z;
    }
    // Compute and add center to clouds
    pcl::PointXYZ center;
    center.x = accx / it->indices.size();
    center.y = accy / it->indices.size();
    center.z = accz / it->indices.size();
    centers_cloud->push_back(center);
  }
}

Eigen::Affine3f getRotationMatrix(Eigen::Vector3f source,
                                  Eigen::Vector3f target)
{
  Eigen::Vector3f rotation_vector = target.cross(source);
  rotation_vector.normalize();
  double theta = acos(source[2] / sqrt(pow(source[0], 2) + pow(source[1], 2) +
                                       pow(source[2], 2)));

  if (DEBUG)
    std::cout << "Rot. vector: " << rotation_vector << " / Angle: " << theta << std::endl;

  Eigen::Matrix3f rotation =
      Eigen::AngleAxis<float>(theta, rotation_vector) * Eigen::Scaling(1.0f);
  Eigen::Affine3f rot(rotation);
  return rot;
}

class Square
{
private:
  pcl::PointXYZ _center;
  std::vector<pcl::PointXYZ> _candidates;
  float _target_width, _target_height, _target_diagonal;

public:
  Square(std::vector<pcl::PointXYZ> candidates, float width, float height)
  {
    _candidates = candidates;
    _target_width = width;
    _target_height = height;
    _target_diagonal = sqrt(pow(width, 2) + pow(height, 2));

    // Compute candidates centroid
    for (unsigned i = 0; i < candidates.size(); ++i)
    {
      _center.x += candidates[i].x;
      _center.y += candidates[i].y;
      _center.z += candidates[i].z;
    }

    _center.x /= candidates.size();
    _center.y /= candidates.size();
    _center.z /= candidates.size();
  }

  float distance(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
  {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
                pow(pt1.z - pt2.z, 2));
  }

  float perimeter()
  { // TODO: It is assumed that _candidates are ordered, it
    // shouldn't
    float perimeter = 0;
    for (int i = 0; i < 4; ++i)
    {
      perimeter += distance(_candidates[i], _candidates[(i + 1) % 4]);
    }
    return perimeter;
  }

  pcl::PointXYZ at(int i)
  {
    assert(0 <= i && i < 4);
    return _candidates[i];
  }

  bool is_valid()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Check if candidates are at 5% of target's diagonal/2 to their centroid
    for (unsigned i = 0; i < _candidates.size(); ++i)
    {
      candidates_cloud->push_back(_candidates[i]);
      float d = distance(_center, _candidates[i]);
      if (fabs(d - _target_diagonal / 2.) / (_target_diagonal / 2.) >
          GEOMETRY_TOLERANCE)
      {
        return false;
      }
    }
    // Check perimeter?
    pcl::PointCloud<pcl::PointXYZ> sorted_centers;
    sorted_centers = sortPatternCenters(candidates_cloud);
    // std::vector<pcl::PointXYZ> sorted_centers;
    // sortPatternCenters(candidates_cloud, sorted_centers);
    float perimeter = 0;
    for (unsigned i = 0; i < sorted_centers.size(); ++i)
    {
      float current_distance = distance(
          sorted_centers[i], sorted_centers[(i + 1) % sorted_centers.size()]);
      if (i % 2)
      {
        if (fabs(current_distance - _target_height) / _target_height >
            GEOMETRY_TOLERANCE)
        {
          return false;
        }
      }
      else
      {
        if (fabs(current_distance - _target_width) / _target_width >
            GEOMETRY_TOLERANCE)
        {
          return false;
        }
      }
      perimeter += current_distance;
    }
    float ideal_perimeter = (2 * _target_width + 2 * _target_height);
    if (fabs((perimeter - ideal_perimeter) / ideal_perimeter >
             GEOMETRY_TOLERANCE))
    {
      return false;
    }

    // Check width + height?
    return true;
  }
};

void comb(int N, int K, std::vector<std::vector<int>> &groups)
{
  int upper_factorial = 1;
  int lower_factorial = 1;

  for (int i = 0; i < K; i++)
  {
    upper_factorial *= (N - i);
    lower_factorial *= (K - i);
  }
  unsigned n_permutations = upper_factorial / lower_factorial;

  if (DEBUG)
    std::cout << N << " centers found. Iterating over " << n_permutations
              << " possible sets of candidates" << std::endl;

  std::string bitmask(K, 1); // K leading 1's
  bitmask.resize(N, 0);      // N-K trailing 0's

  // print integers and permute bitmask
  do
  {
    std::vector<int> group;
    for (int i = 0; i < N; ++i) // [0..N-1] integers
    {
      if (bitmask[i])
      {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  assert(groups.size() == n_permutations);
}

const std::string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

#endif
