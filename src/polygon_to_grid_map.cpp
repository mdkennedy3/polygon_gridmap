#include "ros/ros.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>

#include <yaml-cpp/yaml.h>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "polygon_to_grid_map");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  ros::Publisher point_publisher = nh.advertise<sensor_msgs::PointCloud2>("grid_points", 1, true);

  // Create grid map.
  GridMap map({"layer"});
  map.setFrameId("map");

  //Dimensions of vicon area in m
  double length, width, resolution;
  std::string yaml_file;
  nh.param("length", length, 18.0);
  nh.param("width", width, 6.0);
  nh.param("resolution", resolution, 0.1);
  nh.param<std::string>("yaml_file", yaml_file, "test.yaml");

  map.setGeometry(Length(length, width), resolution, Position(length/2.0, width/2.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  map.move(Position(0,0));

  YAML::Node config = YAML::LoadFile(yaml_file);

  YAML::Node obstaclesNode = config["obstacles"];
    if (obstaclesNode.IsNull()) return false; //Sprite Not Found?

  int total_obstacles = obstaclesNode.size();
  ROS_INFO("yaml size %d", total_obstacles);

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  //Set all cells as unoccupied
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("layer", *it) = 0.0;
  }

  for (unsigned short i = 0; i < total_obstacles; ++i)
  {
    YAML::Node poly = obstaclesNode[i];
    int num_polygon_points = poly.size();
    ROS_INFO("Num pol  %d",num_polygon_points);

    grid_map::Polygon polygon;
    polygon.setFrameId(map.getFrameId());

    for (unsigned short j = 0; j < num_polygon_points; ++j)
    {
      YAML::Node point = poly[j];
      float x = point[0].as<float>();
      float y = point[1].as<float>();
      ROS_INFO("x %g y %g", x, y);
      polygon.addVertex(Position(x, y));
    }

    //Add the first point again to close polygon
    YAML::Node point = poly[0];
    float x = point[0].as<float>();
    float y = point[1].as<float>();
    polygon.addVertex(Position(x,y));

    for (grid_map::PolygonIterator iterator(map, polygon);
        !iterator.isPastEnd(); ++iterator) {
      map.at("layer", *iterator) = 1.0;
    }
  }

  // Convert again to OccupancyGrid msg.
  nav_msgs::OccupancyGrid occupancy_grid;
  GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancy_grid);

  sensor_msgs::PointCloud2 point_cloud;
  GridMapRosConverter::toPointCloud(map, "layer", point_cloud);

  ros::Rate rate(5.0);
  while (nh.ok()) {

    ros::Time time = ros::Time::now();
    occupancy_grid.header.stamp = time;
    publisher.publish(occupancy_grid);

    point_cloud.header.stamp = time;
    point_publisher.publish(point_cloud);

    //ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    rate.sleep();
  }

  return 0;
}