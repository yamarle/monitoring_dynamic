
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// ESTO HABRÁ QUE TOCARLO
//#define MIN_DIST 0.8
#define MIN_DIST 2.0
#define MAX_LINEAR_VEL 0.7
//#define MAX_LINEAR_VEL 0.1
#define MAX_ANGULAR_VEL 3.14
#define MIN_PROXIMITY_RANGE 0.5

#define GOAL_DISTANCE 0.35
#define GOAL_ANGLE 0.05
#define ROBOT_SIZE 0.35

using namespace std;

namespace multi_util{

	// Utilidades
	double length(geometry_msgs::Point p);
	double length(geometry_msgs::Twist v);
	double lengthSquared(geometry_msgs::Point p);
	geometry_msgs::Point normalize(geometry_msgs::Point p);
	geometry_msgs::Twist normalize(geometry_msgs::Twist v);
	geometry_msgs::Point leftNormalVector(geometry_msgs::Point p);
	geometry_msgs::Point rightNormalVector(geometry_msgs::Point p);
	double angleTo(geometry_msgs::Point p1, geometry_msgs::Point p2);
	double normalizeAngle(double angle);
	double polarAngle(geometry_msgs::Point p);
	double toRadian(double angle);
	int sign(double value);

	// Posiciones en el mapa
	geometry_msgs::Pose getRobotPose();
	pair<double, double> getRobotPosition();
	pair<double, double> getRobotPosition(int robot_no);
	pair<double, double> getRobotMapPosition(int robot_no);
	vector<pair<double, double>> getTeamPositions(int team_size);
	pair<int, int> transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> robotPos);
	pair<int, int> transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, geometry_msgs::Pose pose);
	geometry_msgs::Point transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, double x, double y);
	geometry_msgs::Point transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> pos);
	vector<geometry_msgs::Point> transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, vector<geometry_msgs::Pose> poses);

	void toGridCoordinates(double x, double y, const nav_msgs::OccupancyGrid& map, double &gx, double &gy);
	vector<geometry_msgs::Pose> poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map);
	vector<geometry_msgs::Point> points2GridCoordinates(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map);

	geometry_msgs::Pose transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<int, int> robotGridPos);
	geometry_msgs::Pose transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, int x, int y);
	geometry_msgs::Point tfGridPos2MapPoint(const nav_msgs::OccupancyGrid& map, int x, int y);
	vector<geometry_msgs::Point> tfGridPos2MapPos(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y);
	vector<geometry_msgs::PoseStamped> transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y);
	vector<geometry_msgs::PoseStamped> poses2PoseStamped(vector<geometry_msgs::Pose> poses);

	vector<geometry_msgs::Point> buildGraphPoints(vector<int> x, vector<int> y, vector<vector<bool>> graph, nav_msgs::OccupancyGrid map);
	vector<geometry_msgs::Point> buildTreePoints(vector<int> x, vector<int> y, vector<vector<int>> tree, nav_msgs::OccupancyGrid map);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<vector<bool>> graph);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<vector<int>> tree);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<int> branch);

	geometry_msgs::Point32 rotatePoint(geometry_msgs::Pose pose, double x, double y);

	vector<geometry_msgs::Point> footprint(geometry_msgs::Pose pose, double sx, double sy);
	vector<geometry_msgs::Point> circlePoints(double x, double y, double r, double rs);

	bool checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float d, float ad);
	pair<float, float> poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	float pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float orientationDist(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

	// Laser
	void _laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos);
	void laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos);
	void laserPosesCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Pose>& laserPoses);
	vector<geometry_msgs::Pose> laserPosesTf(vector<geometry_msgs::Pose> laserPoses);
	vector<geometry_msgs::Pose> laserPosesTf(geometry_msgs::Pose robotPose, vector<geometry_msgs::Pose> laserPoses);

	// Mapa
	void _occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid);
	void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid);
	void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid, vector<vector<float>>& grid);
	vector<vector<int>> occupancyGridToVector(const nav_msgs::OccupancyGrid& occupancyGridMsg);
	vector<vector<int>> occupancyGrid2Grid(vector<vector<int>> occupancyGrid);
	void occupancyGrid2Grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid, vector<vector<float>>& grid);
	vector<signed char> grid2occupancyGridData(vector<vector<float>> grid);
	vector<signed char> grid2occupancyGridData(vector<vector<int>> grid);
	vector<signed char> map2occupancyGridData(vector<vector<int>> map);
	vector<signed char> map2occupancyGridData(vector<vector<float>> map);

	// Planificación de caminos
	vector<vector<float>> navigableGradient(vector<vector<float>> obst_grad, float size);
	vector<vector<float>> navigableGrid(vector<vector<float>> obst_grad, float size);
	nav_msgs::Path transformToMapPath(vector<int> x, vector<int> y, const nav_msgs::OccupancyGrid& mapMsg);
	vector<geometry_msgs::Pose> convertPointsToPoses(vector<geometry_msgs::Point> points);
	geometry_msgs::Pose computePose(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float computePathDistance(nav_msgs::Path path);

	vector<geometry_msgs::Pose> path2Poses(nav_msgs::Path path);
	pair<float, float> distPose2Path(geometry_msgs::Pose p, nav_msgs::Path path, int evp);

	vector<geometry_msgs::Pose> setFollowerPathPoses(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses);

	// Navegación
	geometry_msgs::Twist computeVelocities(pair<double, double> start, pair<double, double> goal);
	geometry_msgs::Twist computeVelocities(pair<double, double> start, geometry_msgs::Point goal);
	pair<double, double> selectGoalFromPath(nav_msgs::Path path, pair<double, double> currPos);

	void count(vector<vector<int>> grid);

	// Representación
	visualization_msgs::MarkerArray setTreeLines(vector<geometry_msgs::Point> points, vector<vector<int>> tree, vector<float> colors, string map_topic, string tree_frame_id, float resolution);
	void clearTreeLines(visualization_msgs::MarkerArray &lines);

};
