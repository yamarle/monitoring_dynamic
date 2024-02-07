
// Funciones que usa nuestro método
// Utilidades
#include <multi_dynamic/multi_util.h>
#include <multi_dynamic/util/funciones.hpp>
#include <multi_dynamic/util/initialization.hpp>
#include <multi_dynamic/util/fmm_2.hpp>
#include <multi_dynamic/util/path.hpp>
#include <multi_dynamic/util/bresenham.hpp>
#include <multi_dynamic/util/geometry.hpp>
#include <multi_dynamic/util/generic_fmm.hpp>

// Exploración
#include <multi_dynamic/exploration_functions.hpp>
#include <multi_dynamic/dynamic_areas.hpp>

// Para enviar goals al agente
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

// Para sincronizar los mensajes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Para las funciones del laser
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>

// ESTO HABRÁ QUE TOCARLO
//#define MIN_DIST 0.8
#define MIN_DIST 2.0
#define MAX_LINEAR_VEL 0.7
//#define MAX_LINEAR_VEL 0.1
#define MAX_ANGULAR_VEL 3.14

using namespace std;

// PÁRAMETROS DE ENTRADA
double sX, sY; // Tamaño del robot
string map_topic, pose_topic, laser_topic, pathToFollow_topic;
string robot_frame;
double laser_range;

// Posiciones del laser del explorador
sensor_msgs::LaserScan laserMsg; // El mensaje inicial de ROS
vector<geometry_msgs::Point> laserPos, laserGridPos;
vector<geometry_msgs::Pose> laserPoses, laserGridPoses;
Poss<int> laserPoss;

nav_msgs::OccupancyGrid mapMsg;

class LaserScanToPointCloud{

public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    string mapTopic;
    LaserScanToPointCloud(ros::NodeHandle n, string mapTopic) : 
        n_(n),
        laser_sub_(n_, laser_topic, 10), // queue size (cantidad de mensajes almacenados)
        laser_notifier_(laser_sub_, listener_, robot_frame, 10) // queue size (cantidad de mensajes almacenados)
    {
        this->mapTopic = mapTopic;
        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;

        laserMsg = *scan_in; // Almaceno el propio mensaje

        try{
            projector_.transformLaserScanToPointCloud(mapTopic, *scan_in, cloud, listener_, laserMsg.range_max + 0.00001);
            //projector_.transformLaserScanToPointCloud(mapTopic, laserMsg, cloud, listener_, laserMsg.range_max + 0.00001);
        }catch (tf::TransformException& e){
            cout << e.what();
            return;
        }

        // Do something with cloud.

        laserPoses.clear();
        laserGridPoses.clear();
        laserGridPos.clear();
        laserPoss.clear();

        float angle;
        geometry_msgs::Pose pose;

        for(int i=0; i<cloud.points.size(); i++){
            ////cout<<cloud.points[i].x<<", "<<cloud.points[i].y<<", "<<cloud.points[i].z<<endl;
            // Formar las variables refenretes al laser
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = cloud.points[i].x;
            pose.position.y = cloud.points[i].y;
            pose.position.z = cloud.points[i].z;

            // Creo el quaternion
            angle = laserMsg.angle_min + i * laserMsg.angle_increment;
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            laserPoses.push_back(pose);

            laserGridPoses.push_back(pose);
            laserGridPos.push_back(multi_util::transform2GridCoordinates(mapMsg, pose.position.x, pose.position.y));
            laserPoss.push(laserGridPos[laserGridPos.size()-1].y, laserGridPos[laserGridPos.size()-1].x);

        }

    }
};

int main(int argc, char** argv) {
	if (argc < 2) {
		ROS_ERROR("You must specify leader robot id.");
		return -1;
	}

	char *leader_id = argv[1];

	ros::init(argc, argv, "relays");
	ros::NodeHandle nh;


	LaserScanToPointCloud lstopc(nh, "map");


	return 0;
}