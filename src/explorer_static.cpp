
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

#include "ros/master.h"
#include "ros/xmlrpc_manager.h"
#include "std_srvs/Empty.h"

// TODAS LAS VARIABLES GLOBALES QUE VA A USAR EL EXPLORADOR

// PÁRAMETROS DE ENTRADA
double sX, sY; // Tamaño del robot
string map_topic, pose_topic, laser_topic, pathToFollow_topic;
string robot_frame;
double laser_range;

double occup_thres = 0;
bool forget_dynamic_areas = true;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // Para enviar los objetivos al robot

// Posiciones que usa el robot
pair<int, int> gridPosition, goalGridPosition, waypointGrid;
Poss<int> currPoss;
geometry_msgs::PoseWithCovarianceStamped realPose;
geometry_msgs::Pose currPose, goalPose, waypointPose;
geometry_msgs::Point explPoint, goalPoint, wayPoint;

nav_msgs::Odometry robotOdomPoseMsg, _robotOdomPoseMsg;

// Posiciones del laser del explorador
sensor_msgs::LaserScan laserMsg; // El mensaje inicial de ROS
vector<geometry_msgs::Point> laserPos, laserGridPos;
vector<geometry_msgs::Pose> laserPoses, laserGridPoses;
Poss<int> laserPoss;

// El grid del mapa (de ocupación → el que uso para planificar mis cosas: FMM, segmentar, path planning ... )
vector<vector<float>> grid;
int gsx, gsy;

nav_msgs::OccupancyGrid mapMsg;

sensor_msgs::PointCloud laserPointCloud;

// Mapa de lo explorado/no explorado
vector<vector<int>> explored_map;

// Cosas para el planificador/navegador
// Cosas para el planificador/navegador
double maxLinearVel = 0.65;
double maxAngularVel = 1.0;
double robotRadius = 0.5;

int robotRadiusGrid;

// Cosas para la localización
bool bad_localization = true;
float permissive_rotation_velocity = 0.25;

// Variables para los resultados
nav_msgs::Path totalPath; geometry_msgs::PoseStamped totalPathPose;
float missionTime = 0; // Tiempo que se emplea para terminar la misión
float max_mission_time = 600; // Tiempo máximo que se considera para terminar la missión
float precomputationTime; // Tiempo que se ha requerido para realizar los cálculos offline (gradientes, segmentar entorno, ...)
string result_filename = "resultado.txt";
ros::Time beginTime, endTime;

// (v,w) que se calculan para ir al goal
double linearVel = 0.0;
double angularVel = 0.0;
double linearAcc = maxLinearVel/10; // (Valor de la página de ROS: 2.5)
double angularAcc = maxAngularVel/10; // (Valor de la página de ROS: 3.2)
geometry_msgs::Twist cmd_vel; // Comandos de velocidad que se calculan
geometry_msgs::Twist _cmd_vel; // Comandos de velocidad anteriores

Poss<float> path2Poss(nav_msgs::Path path)
{
    Poss<float> res;
    for(int i=0; i<path.poses.size(); i++)
        res.push(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    return res;
}

Poss<float> points2PossF(vector<geometry_msgs::Point> points)
{
    Poss<float> res;
    for(int i=0; i<points.size(); i++)
        res.push(points[i].y, points[i].x);
    return res;
}

Poss<int> points2Poss(vector<geometry_msgs::Point> points)
{
    Poss<int> res;
    for(int i=0; i<points.size(); i++)
        res.push(points[i].y, points[i].x);
    return res;
}

Poss<int> poses2Poss(vector<geometry_msgs::Pose> points)
{
    Poss<int> res;
    for(int i=0; i<points.size(); i++)
        res.push(points[i].position.y, points[i].position.x);
    return res;
}

void showPoses(vector<geometry_msgs::Pose> poses)
{
    for(int i = 0; i<poses.size(); i++){
        cout<<"("<<poses[i].position.x<<", "<<poses[i].position.y<<", "<<poses[i].orientation.w<<")"<<endl;
    }
}

Poss<int> agentPoss(int x, int y, int sx, int sy)
{
    Poss<int> res;
    for(int i=x-sx/2; i<=x+sx/2; i++)
        for(int j=y-sy/2; j<=y+sy/2; j++)
            res.push(i,j);
    return res; 
}

bool allPossInMap(Poss<int> poss, int sx, int sy)
{
    for(int i=0; i<poss.x.size(); i++)
        if(poss.x[i]<0 || poss.x[i]>=sx || poss.y[i]<0 || poss.y[i]>=sy)
            return false;
    return true;
}

vector<geometry_msgs::Point> poss2Points(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& map)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    for(int i=0; i<poss.size(); i++){
        for(int j=0; j<poss[i].x.size(); j++){
            p.y = poss[i].x[j] * map.info.resolution + map.info.origin.position.x;
            p.x = poss[i].y[j] * map.info.resolution + map.info.origin.position.y;
            res.push_back(p);
        }
    }

    return res;
}

Poss<int> allLaserPoss(int x, int y, Poss<int> laserPoss)
{
    Poss<int> res;

    for(int i=0; i<laserPoss.x.size(); i++){
        //cout<<laserPoss.x[i]<<", "<<laserPoss.y[i]<<endl;
        Poss<int> p = bresenham::points(x, y, laserPoss.x[i], laserPoss.y[i]);
        for(int j=0; j<p.x.size(); j++){
            res.push(p.x[j], p.y[j]);
        }
    }

    return res;

}

void showPoints(vector<geometry_msgs::Point> points)
{
    for(int i=0; i<points.size(); i++)
        cout<<points[i].x<<", "<<points[i].y<<endl;
}

vector<geometry_msgs::Point> poses2Points(vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::Point> res(poses.size());
    for(int i=0; i<poses.size(); i++){
        res[i].x = poses[i].position.x;
        res[i].y = poses[i].position.y;
        res[i].z = 0;
    }
    return res;
}

sensor_msgs::PointCloud grid2Pointcloud(vector<vector<float>> grid, const nav_msgs::OccupancyGrid& map)
{
    sensor_msgs::PointCloud res;
    geometry_msgs::Point32 p;

    //res.header.seq = 0;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = map_topic;
    for(int i=0; i<grid.size(); i++){
        for(int j=0; j<grid[i].size(); j++){
            p.x = j * map.info.resolution + map.info.origin.position.x;
            p.y = i * map.info.resolution + map.info.origin.position.y;
            p.z = grid[i][j];
            res.points.push_back(p);
        }
    }

    return res;   
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
    //ros::Duration(1.0).sleep();
    //cout<<"SUSCRITO CON AMCL"<<endl;
    currPose = robotPoseMsg->pose.pose;
    gridPosition = multi_util::transformToGridCoordinates(mapMsg, currPose);
    currPoss.x[0] = gridPosition.first; currPoss.y[0] = gridPosition.second;

    totalPathPose.pose = currPose;
    totalPath.poses.push_back(totalPathPose);

}

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    //ros::Duration(1.0).sleep();
    //cout<<"SUSCRITO CON odom"<<endl;
    currPose = robotPoseMsg->pose.pose;
    gridPosition = multi_util::transformToGridCoordinates(mapMsg, currPose);
    currPoss.x[0] = gridPosition.first; currPoss.y[0] = gridPosition.second;

    robotOdomPoseMsg = *robotPoseMsg;

    totalPathPose.pose = currPose;
    totalPath.poses.push_back(totalPathPose);

}

//****************************************************************************************************************
class _LaserScanToPointCloud{

public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    string mapTopic;
    _LaserScanToPointCloud(ros::NodeHandle n, string mapTopic) : 
        n_(n),
        laser_sub_(n_, laser_topic, 10),
        laser_notifier_(laser_sub_, listener_, robot_frame, 10)
    {
        this->mapTopic = mapTopic;
        laser_notifier_.registerCallback(boost::bind(&_LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;
        laserMsg = *scan_in; // Almaceno el propio mensaje
        try{
            projector_.transformLaserScanToPointCloud(mapTopic, *scan_in, cloud, listener_);
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

        ////cout<<cloud.points.size()<<" puntos"<<endl;
        for(int i=0; i<cloud.points.size(); i++){
            ////cout<<cloud.points[i].x<<", "<<cloud.points[i].y<<", "<<cloud.points[i].z<<endl;
            // Formar las variables refenretes al laser
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = cloud.points[i].x;
            pose.position.y = cloud.points[i].y;
            pose.position.z = cloud.points[i].z;

            // Creo el quaternion
            angle = scan_in->angle_min + i * scan_in->angle_increment;
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            laserPoses.push_back(pose);

            laserGridPoses.push_back(pose);
            laserGridPos.push_back(multi_util::transform2GridCoordinates(mapMsg, pose.position.x, pose.position.y));
            laserPoss.push(laserGridPos[laserGridPos.size()-1].y, laserGridPos[laserGridPos.size()-1].x);

        }

    }
};


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
        laser_sub_(n_, laser_topic, 10),
        laser_notifier_(laser_sub_, listener_, robot_frame, 10)
    {
        this->mapTopic = mapTopic;
        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    sensor_msgs::PointCloud computePoints()
    {
        sensor_msgs::PointCloud res;
        res.header.stamp = ros::Time::now();
        res.header.frame_id = robot_frame;
        //res.header.frame_id = map_topic;
        geometry_msgs::Point32 p;
        float angle;
        for(int i=0; i<laserMsg.ranges.size(); i++){
            angle = laserMsg.angle_min + i * laserMsg.angle_increment;
            //angles.push_back(angle);
            p.x = currPose.position.x + laserMsg.ranges[i] * cos(angle);
            p.y = currPose.position.y + laserMsg.ranges[i] * sin(angle);
            p.z = 0.0;
            res.points.push_back(p);
        }
        return res;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;
        laserMsg = *scan_in; // Almaceno el propio mensaje

        // Do something with cloud.

        /*
        cout<<"DENTRO DEL LASER CALLBACK: "<<robot_frame<<" - "<<laser_range<<endl;

        cout<<cloud.points.size()<<" - "<<laserMsg.ranges.size()<<" | "<<laserMsg.intensities.size()<<" puntos del laser"<<endl;
        cout<<laserMsg.range_min<<" - "<<laserMsg.range_max<<endl;

        cout<<" INTENSITIES: ";
        for(int i=0; i<laserMsg.ranges.size(); i++)
            cout<<laserMsg.ranges[i]<<" ";
        cout<<endl;
        */

        cloud = computePoints();

        laserPoses.clear();
        laserGridPoses.clear();
        laserGridPos.clear();
        laserPoss.clear();

        geometry_msgs::Pose pose;

        for(int i=0; i<cloud.points.size(); i++){
            // Formar las variables referentes al laser
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = cloud.points[i].x;
            pose.position.y = cloud.points[i].y;
            pose.position.z = cloud.points[i].z;

            // Creo el quaternion
            //float angle = scan_in->angle_min + i * scan_in->angle_increment;
            //pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            // Insertar los puntos

            laserPoses.push_back(pose);

            laserGridPoses.push_back(pose);
            laserGridPos.push_back(multi_util::transform2GridCoordinates(mapMsg, pose.position.x, pose.position.y));
            laserPoss.push(laserGridPos[laserGridPos.size()-1].y, laserGridPos[laserGridPos.size()-1].x);

        }

    }
};

Poss<int> newObstacles(Poss<int> poss, vector<float> map_values, vector<vector<float>> navMap)
{
    Poss<int> res;
    for(int i=0; i<poss.x.size(); i++)
        if(navMap[poss.x[i]][poss.y[i]] != map_values[i]) res.push(poss.x[i], poss.y[i]);
    return res;
}

Poss<int> newObstacles(Poss<int> poss, vector<float> map_values)
{
    Poss<int> res;
    for(int i=0; i<poss.x.size(); i++)
        if(map_values[i]) res.push(poss.x[i], poss.y[i]);
    return res;
}

Poss<int> newValuesinMap(Poss<int> poss, vector<float> values, vector<vector<float>> original_map, vector<float> &new_values)
{
    // Variables:
    // - poss: posiciones del grid que se ven
    // - values: lo que se ve en esas posiciones
    // - original_map: el mapa original que contiene unicamente los obstáculos estáticos

    // - res/new_values: posiciones en las que hay cambios
    Poss<int> res;
    for(int i=0; i<poss.x.size(); i++)
        if(original_map[poss.x[i]][poss.y[i]] && values[i] == 0){ // Un obstáculo dinámico
            res.push(poss.x[i], poss.y[i]);
            new_values.push_back(0);
        }else if(original_map[poss.x[i]][poss.y[i]] && values[i]){ // había un obstáculo dinámico que se ha movido
            res.push(poss.x[i], poss.y[i]);
            new_values.push_back(original_map[poss.x[i]][poss.y[i]]);
        }
    return res;
}

void updateNavigableMap(vector<vector<float>> &navigable_map, vector<vector<float>> original_map, Poss<int> laser_poss, Poss<int> all_laser_poss, int ax, int ay)
{
        // Todos los puntos del laser (bresenham)
    bool b;
    for(int i=0; i<all_laser_poss.x.size(); i++){
        b = true;
        for(int j=0; j<laser_poss.x.size(); j++){
            if(all_laser_poss.x[i] == laser_poss.x[j] && all_laser_poss.y[i] == laser_poss.y[j]){
                b = false;
            }
        }
        if(b){
            navigable_map[all_laser_poss.x[i]][all_laser_poss.y[i]] = original_map[all_laser_poss.x[i]][all_laser_poss.y[i]];
        }
    }

    // Los puntos visibles con el laser son obstáculos
    for(int i=0; i<laser_poss.x.size(); i++){
        //if(navigable_map[laser_poss.x[i]][laser_poss.y[i]]){
            navigable_map[laser_poss.x[i]][laser_poss.y[i]] = 0;
            // Incluyo las posiciones del agente (volumen)
            Poss<int> agent_vol = agentPoss(laser_poss.x[i], laser_poss.y[i], ax, ay);
            agent_vol = fix_poss_in_map(agent_vol, original_map.size(), original_map[0].size());
            for(int j=0; j<agent_vol.x.size(); j++){
                navigable_map[agent_vol.x[j]][agent_vol.y[j]] = 0;
            }
        //}
    }

}

Poss<int> dynObstPos(Poss<int> laser_poss, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> grid, vector<geometry_msgs::Point> &obst_pos)
{
    Poss<int> res;

    for(int i=0; i<laser_poss.x.size(); i++){
        if(grid[laser_poss.x[i]][laser_poss.y[i]]){
            res.push(laser_poss.x[i], laser_poss.y[i]);
            obst_pos.push_back(laser_poses[i].position);
        }
    }

    return res;
}

Poss<int> dynObstPos(Poss<int> laser_poss, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> original_grid, Poss<int> all_laser_poss, vector<vector<float>> &curr_grid, vector<geometry_msgs::Point> &obst_pos)
{
    Poss<int> res;

    for(int i=0; i<laser_poss.x.size(); i++){
        if(original_grid[laser_poss.x[i]][laser_poss.y[i]]){
            res.push(laser_poss.x[i], laser_poss.y[i]);
            obst_pos.push_back(laser_poses[i].position);
        }
    }

    for(int i=0; i<all_laser_poss.x.size(); i++){
        if(!curr_grid[all_laser_poss.x[i]][all_laser_poss.y[i]]){
            curr_grid[all_laser_poss.x[i]][all_laser_poss.y[i]] = original_grid[all_laser_poss.x[i]][all_laser_poss.y[i]];
        }
    }

    return res;
}

vector<int> number_of_obstacles_in_segments_zs(Poss<int> obstacles, vector<vector<int>> segments, int nsegs)
{
    vector<int> res(nsegs,0);
    if(!obstacles.x.size()) return res;
    for(int i=0; i<obstacles.x.size(); i++){
        if(segments[obstacles.x[i]][obstacles.y[i]] >= 0) res[segments[obstacles.x[i]][obstacles.y[i]]]++;
    }
    return res;
} // Cantidad de obstáculos por segmentos (los segmentos empiezan por "0")

vector<int> possInSegments(Poss<int> poss, vector<vector<int>> segmentsMap, int nsegments)
{
    vector<bool> segments(nsegments, false);
    for(int i=0; i<poss.x.size(); i++)
        if(segmentsMap[poss.x[i]][poss.y[i]] >= 0 && segmentsMap[poss.x[i]][poss.y[i]] < nsegments-1) segments[segmentsMap[poss.x[i]][poss.y[i]]] = true;
    return bool2ind(segments);   
}

vector<int> possInSegments(vector<Poss<int>> poss, vector<vector<int>> segmentsMap, int nsegments)
{
    vector<bool> segments(nsegments, false);
    for(int i=0; i<poss.size(); i++)
        for(int j=0; j<poss[i].x.size(); j++)
            if(segmentsMap[poss[i].x[j]][poss[i].y[j]] >= 0 && segmentsMap[poss[i].x[j]][poss[i].y[j]] < nsegments-1) segments[segmentsMap[poss[i].x[j]][poss[i].y[j]]] = true;
    return bool2ind(segments);   
}

vector<Poss<int>> clusterPoss(Poss<int> poss, vector<vector<int>> segmentsMap, int nsegments, vector<int> &segs)
{
    vector<Poss<int>> res;
    segs.clear();
    bool f;
    for(int i=0; i<poss.x.size(); i++){
        if(segmentsMap[poss.x[i]][poss.y[i]] >= 0 && segmentsMap[poss.x[i]][poss.y[i]] < nsegments-1){
            f = false;
            for(int j=0; j<segs.size(); j++){
                if(segmentsMap[poss.x[i]][poss.y[i]] == segs[j]){
                    res[j].push(poss.x[i], poss.y[i]);
                    f = true;
                    break;
                }
            }
            if(!f){
                res.resize(res.size()+1);
                res[res.size()-1].push(poss.x[i], poss.y[i]);
                segs.push_back(segmentsMap[poss.x[i]][poss.y[i]]);
            }
        }
    }
    return res;   
} // Separar un conjunto de puntos por segmentos que atraviesan

dynamic_areas::Areas segs2areas(vector<int> segs, fmm_segment::Segments segments)
{
    dynamic_areas::Areas res;

    res.N = segs.size();
    res.centroids(res.N); res.real_centroids(res.N);
    res.contours.resize(res.N); res.contours_edges.resize(res.N);
    res.poss.resize(res.N);
    res.map.resize(segments.map.size(), vector<int>(segments.map[0].size(),-1));
    for(int i=0; i<segs.size(); i++){
        res.centroids.x[i] = res.real_centroids.x[i] = segments.centroids.x[segs[i]];
        res.centroids.y[i] = res.real_centroids.y[i] = segments.centroids.y[segs[i]];
        res.contours[i] = segments.contour_poss[segs[i]];
        res.contours_edges[i] = segments.contour_poss[segs[i]];
        res.poss[i] = segments.poss[segs[i]];
        for(int j=0; j<res.poss[i].x.size(); j++){
            res.map[res.poss[i].x[j]][res.poss[i].y[j]] = i;
        }
        for(int j=0; j<res.contours[i].x.size(); j++){
            res.map[res.contours[i].x[j]][res.contours[i].y[j]] = i;
        }
    }

    return res;
} // Transformar los segmentos en el objeto de áreas dinámicas

vector<int> visibleAreas(Poss<int> visiblePoss, vector<vector<bool>> trajectoriesMap, vector<vector<int>> areasMap, int nareas)
{
    vector<bool> areas(nareas, false);
    for(int i=0; i<visiblePoss.x.size(); i++){
        if(trajectoriesMap[visiblePoss.x[i]][visiblePoss.y[i]]){ // Estoy viendo un obstáculo dentro de una de las trayectorías que tengo almacenadas
            if(areasMap[visiblePoss.x[i]][visiblePoss.y[i]] >= 0 && areasMap[visiblePoss.x[i]][visiblePoss.y[i]] < nareas-1) areas[areasMap[visiblePoss.x[i]][visiblePoss.y[i]]] = true;
        }
    }
    return bool2ind(areas);
}

void assign_value(Poss<int> poss, float val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] = val;
} // Asignar un valor a las posiciones de un grid

void increase_value(Poss<int> poss, float val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] += val;
} // Asignar un valor a las posiciones de un grid

vector<vector<float>> navigable_grid_occup_thres(vector<vector<float>> static_grid, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
{
    vector<vector<float>> res = static_grid;
    int val = 0, ap;
    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);

    assign_value(free_poss, val + 5, res); // Las posiciones libres tienen prioridad

    for(int i=0; i<number_of_obstacles.size(); i++){
        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
        //cout<<(float)number_of_obstacles[i]/ap<<" <= "<<occupancy_threshold<<endl;
        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
            assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
        }else{ // área no navegable
            if(i != my_segment)
                assign_value(areas_poss[i], 0, res);
            else
                assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
        }
    }

    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

    return res;
} // Construcción del grid de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

vector<int> obstacles_sort(vector<int> no)
{
    int n = no.size();
    int max, ind;
    vector<int> res(n, 0);
    for(int i=0; i<n; i++){
        max = 0;
        for(int j=0; j<n; j++){
            if(max < no[j]){
                max = no[j];
                ind = j;
            }
        }
        if(max){
            no[ind] = 0;
            res[ind] = i + 1;
        }
    }
    return res;
}

vector<vector<float>> navigable_norm_grad_occup_thres(vector<vector<float>> norm_grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
{
    // norm_grad: gradiente de obstáculos normalizado [0- 1]
    vector<vector<float>> res = norm_grad;
    int val = 0, ap;
    //number_of_obstacles = norm_v(number_of_obstacles);
    number_of_obstacles = obstacles_sort(number_of_obstacles);
    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);
    else return res;

    /*
    cout<<"Cantidad de obstáculos: "<<number_of_obstacles.size()<<" - "<<areas_poss.size()<<endl;
    vector<float> Amin(number_of_obstacles.size(), INFINITY), Amax(number_of_obstacles.size(), 0);
    float max = 0;
    vector<int> no(number_of_obstacles.size(), 0);
    
    // Los valores más altos son a los que va a tender el camino
    // Los valores más bajos serán zonas a evitar -> áreas
    
    for(int i=0; i<areas_poss.size(); i++){
        no[i] = areas_poss[i].x.size();
        for(int j=0; j<areas_poss[i].x.size(); j++){
            if(norm_grad[areas_poss[i].x[j]][areas_poss[i].y[j]] < Amin[i]) Amin[i] = norm_grad[areas_poss[i].x[j]][areas_poss[i].y[j]];
            if(norm_grad[areas_poss[i].x[j]][areas_poss[i].y[j]] > Amax[i]) Amax[i] = norm_grad[areas_poss[i].x[j]][areas_poss[i].y[j]];
            if(max < Amax[i]) max = Amax[i];
        }
    }

    //sh_vect_h(Amin, "Amin");
    //sh_vect_h(Amax, "Amax");
    sh_vect_h(no, "no");
    sh_vect_h(number_of_obstacles, "no");

    
    cout<<"máximo: "<<max<<" - "<<val<<endl; // El máximo no es el primero
    */

    //increase_value(free_poss, val + 1, res); // Las posiciones libres tienen prioridad
    increase_value(free_poss, 1, res); // Las posiciones libres tienen prioridad

    //cout<<"area libre: "<<val + 5<<endl;

    for(int i=0; i<number_of_obstacles.size(); i++){
        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
            //assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
            //increase_value(areas_poss[i], max - Amax[i] + 0.0001, res);
            //increase_value(areas_poss[i], val - number_of_obstacles[i], res);
            increase_value(areas_poss[i], 0, res);
            //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
        }else{ // área no navegable
            if(i != my_segment){
                assign_value(areas_poss[i], 0, res);
            }else{
                //increase_value(areas_poss[i], max - Amax[i] + 0.0001, res);
                //increase_value(areas_poss[i], val - number_of_obstacles[i], res);
                increase_value(areas_poss[i], 0, res);
                //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
            }
        }
    }

    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

    return res;
} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

vector<vector<float>> navigable_grad_occup_thres(vector<vector<float>> grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
{
    // norm_grad: gradiente de obstáculos normalizado [0- 1]
    vector<vector<float>> res = grad;
    int val = 0, ap;
    //number_of_obstacles = norm_v(number_of_obstacles);
    number_of_obstacles = obstacles_sort(number_of_obstacles);
    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);
    else return res;

    vector<float> no(number_of_obstacles.size(), 0);
    val = max_v(number_of_obstacles);
    for(int i=0; i<no.size(); i++) no[i] = (float)number_of_obstacles[i]/val;

    val = 2;

    // Las posiciones libres tienen prioridad
    //increase_value(free_poss, val + 1, res); // Las posiciones libres tienen prioridad
    increase_value(free_poss, val + 1, res); // Subir el gradiente por encima de los obstáculos

    for(int i=0; i<number_of_obstacles.size(); i++){
        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
            //assign_value(areas_poss[i], val - number_of_obstacles[i], res);
            assign_value(areas_poss[i], val - no[i], res);
            //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
        }else{ // área no navegable
            if(i != my_segment){
                assign_value(areas_poss[i], 0, res);
            }else{
                //assign_value(areas_poss[i], val - number_of_obstacles[i], res);
                assign_value(areas_poss[i], val - no[i], res);
                //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
            }
        }
    }

    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

    return res;
} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

vector<vector<float>> navigable_grad_occup_thres(vector<vector<float>> grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<Poss<int>> areas_contours, int rx, int ry, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
{
    // norm_grad: gradiente de obstáculos normalizado [0- 1]
    vector<vector<float>> res = grad;
    int val = 0, ap;
    //number_of_obstacles = norm_v(number_of_obstacles);
    number_of_obstacles = obstacles_sort(number_of_obstacles);
    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);
    else return res;

    vector<float> no(number_of_obstacles.size(), 0);
    val = max_v(number_of_obstacles);
    for(int i=0; i<no.size(); i++) no[i] = (float)number_of_obstacles[i]/val;

    val = 2;

    // Las posiciones libres tienen prioridad
    //increase_value(free_poss, val + 1, res); // Las posiciones libres tienen prioridad
    increase_value(free_poss, val + 1, res); // Subir el gradiente por encima de los obstáculos

    Poss<int> vol;
    for(int i=0; i<number_of_obstacles.size(); i++){
        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
            //assign_value(areas_poss[i], val - number_of_obstacles[i], res);
            assign_value(areas_poss[i], val - no[i], res);
            for(int j=0; j<areas_contours[i].x.size(); j++){
                vol = agentPoss(areas_contours[i].x[j], areas_contours[i].y[j], rx, ry);
                assign_value(vol, val - no[i], res);
            }
            //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
        }else{ // área no navegable
            if(i != my_segment){
                assign_value(areas_poss[i], 0, res);
            }else{
                //assign_value(areas_poss[i], val - number_of_obstacles[i], res);
                assign_value(areas_poss[i], val - no[i], res);
                for(int j=0; j<areas_contours[i].x.size(); j++){
                    vol = agentPoss(areas_contours[i].x[j], areas_contours[i].y[j], rx, ry);
                    assign_value(vol, val - no[i], res);
                }
                //cout<<"area "<<i<<": "<<val - number_of_obstacles[i]<<" | "<<val<<endl;
            }
        }
    }

    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

    return res;
} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

Poss<int> join_obstacle_free_poss(vector<Poss<int>> poss, vector<bool> obstacles_in_areas)
{
    Poss<int> res;
    if(poss.size() != obstacles_in_areas.size()) return res;
    for(int i=0; i<poss.size(); i++)
        if(!obstacles_in_areas[i])
            res.append(poss[i]);
    return res;
}

Path runawayPath(int x, int y, vector<vector<float>> navigableArea, vector<vector<int>> segments_map, vector<bool> dynamism_in_segments, vector<Poss<int>> segments_poss)
{
    Path res;

    cout<<dynamism_in_segments.size()<<" - "<<segments_poss.size()<<endl;
    Poss<int> point, poss = join_obstacle_free_poss(segments_poss, dynamism_in_segments);
    FMM gr(x, y, navigableArea);
    vector<vector<float>> grad = gr.pos_coverage(1, poss, point);
    if(point.x.size()){
        res.gradient_descent_(grad, point.x[0], point.y[0]);
    }

    cout<<"camino encontrado: "<<res.tam<<" - "<<res.x.size()<<endl;

    return res;
}

void erase_trajectories_points(Poss<int> pos, vector<vector<bool>> &map, Poss<int> &traj_points, int &nvisited_points)
{
    for(int i=0; i<pos.x.size(); i++)
        if(map[pos.x[i]][pos.y[i]] == true){
            traj_points.erase_point(pos.x[i], pos.y[i]); // Eliminar punto
            map[pos.x[i]][pos.y[i]] = false; // Marcar en el mapa
            nvisited_points--;
        }
}

//****************************************************************************************************************

bool updateMap = true;
void checkUpdate(geometry_msgs::Twist cmdMsg)
{
    if(cmdMsg.linear.x || cmdMsg.linear.y || cmdMsg.linear.z || cmdMsg.angular.x || cmdMsg.angular.y || cmdMsg.angular.z) updateMap = false;
    else updateMap = true;
}

bool moving = false;
void checkMov(nav_msgs::Odometry odomMsg)
{
    if(odomMsg.twist.twist.linear.x || odomMsg.twist.twist.angular.z) moving = true;
    else moving = false;
}

pair<float, float> poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    pair<float, float> res;
    res.first = sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
    float o1 = tf::getYaw(p1.orientation);
    float o2 = tf::getYaw(p2.orientation);
    res.second = abs(atan2(sin(o1-o2), cos(o1-o2)));
    return res;
}

bool checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float d, float ad)
{
    pair<float, float> dist = poseDistance(p1,p2);
    if(dist.first<=d && dist.second<=ad) return true;
    return false;
}

float distance2closest_obstacle(int x, int y, Poss<int> obstacles)
{
    float res = INFINITY, d;
    for(int i=0; i<obstacles.x.size(); i++){
        d = hypot(x - obstacles.x[i], y - obstacles.y[i]);
        if(res > d) res = d;
    }
    return res;
}

vector<float> normalize_tree_costs(vector<vector<int>> tree, vector<float> tree_costs)
{
    vector<float> res(tree.size(), 0);
    float max = 0;
    //float min = INF;
    for(int i=0; i<tree.size(); i++){
        // Los costes son de los vértices → saco la media de los 2 vértices (arista)
        //res[i] = (tree_costs[tree[i][0]] + tree_costs[tree[i][1]])/2;
        res[i] = tree_costs[tree[i][0]];
        if(max < res[i]) max = res[i];
        //if(min > res[i]) min = res[i];
    }
    
    for(int i=0; i<res.size(); i++){
        res[i]/=max;
        //if(res[i]) res[i]/=max;
        //else res[i] = 0;
    }
    
    //for(int i=0; i<res.size(); i++) res[i] = 1 - res[i]/max;
    return res;
}

visualization_msgs::MarkerArray setTreeLines(vector<geometry_msgs::Point> points, vector<vector<int>> tree, vector<float> colors, string map_topic, string tree_frame_id)
{
    visualization_msgs::MarkerArray res;

    res.markers.resize(tree.size());
    for(int i=0; i<tree.size(); i++){
        res.markers[i].id = i;
        res.markers[i].header.seq = i;
        res.markers[i].header.stamp = ros::Time::now();
        res.markers[i].header.frame_id = map_topic;
        res.markers[i].ns = tree_frame_id;
        res.markers[i].action = visualization_msgs::Marker::ADD;
        res.markers[i].pose.orientation.w = 1;
        res.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        res.markers[i].scale.x = res.markers[i].scale.y = mapMsg.info.resolution;
        //res.markers[i].color.r = colors[i];
        res.markers[i].color.b = colors[i];
        res.markers[i].color.r = colors[i];
        res.markers[i].color.g = colors[i];
        res.markers[i].color.a = 1.0;

        res.markers[i].points.push_back(points[tree[i][0]]);
        res.markers[i].points.push_back(points[tree[i][1]]);
    }

    return res;
}

// ***************************************************************************************************************************************

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer");
    ros::NodeHandle nh;

    // Get tf_prefix from the parameter server
    string tf_prefix;
    nh.getParam("tf_prefix", tf_prefix);

    ros::Rate loopRate(500);

    if(argc == 14){
        map_topic = argv[1];
        pose_topic = argv[2];
        robot_frame = argv[3];
        laser_topic = argv[4];
        pathToFollow_topic = argv[5];
        sX = atof(argv[6]);
        sY = atof(argv[7]);
        laser_range = atof(argv[8]);
        maxLinearVel = atof(argv[9]);
        maxAngularVel = atof(argv[10]);
        occup_thres = atof(argv[11]);
        forget_dynamic_areas = atoi(argv[12]) == 1;
        max_mission_time = atof(argv[13]);
    }else if(argc == 15){
        map_topic = argv[1];
        pose_topic = argv[2];
        robot_frame = argv[3];
        laser_topic = argv[4];
        pathToFollow_topic = argv[5];
        sX = atof(argv[6]);
        sY = atof(argv[7]);
        laser_range = atof(argv[8]);
        maxLinearVel = atof(argv[9]);
        maxAngularVel = atof(argv[10]);
        occup_thres = atof(argv[11]);
        forget_dynamic_areas = atoi(argv[12]) == 1;
        max_mission_time = atof(argv[13]);
        result_filename = argv[14];
    }

    cout<<argc<<" argumentos"<<endl;
    cout<<"map topic:   "<<map_topic<<endl;
    cout<<"pose topic:  "<<pose_topic<<endl;
    cout<<"robot frame: "<<robot_frame<<endl;
    cout<<"laser topic: "<<laser_topic<<endl;
    cout<<"path topic:  "<<pathToFollow_topic<<endl;
    cout<<"tamaño:      "<<sX<<" x "<<sY<<endl;
    cout<<"rango laser: "<<laser_range<<endl;
    cout<<"vel lineal:  "<<maxLinearVel<<endl;
    cout<<"vel angular: "<<maxAngularVel<<endl;
    cout<<"occup thres: "<<occup_thres<<endl;
    cout<<"forget obst: "<<forget_dynamic_areas<<endl;
    cout<<"max time:    "<<max_mission_time<<endl;
    //return 0;

    robotRadius = hypot(sX, sY);

    currPose.position.x = -INF;
    currPoss(1);

    //nav_msgs::OccupancyGrid map;
    vector<vector<int>> occGrid;
    Poss<int> obst;
    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::gridCallback, _1, boost::ref(mapMsg), boost::ref(occGrid), boost::ref(grid)));

    // Subscriber laser
    _LaserScanToPointCloud lstopc(nh, map_topic);
    ros::Subscriber poseSub;
    if(pose_topic.find("amcl")>=0 && pose_topic.find("amcl")<pose_topic.size()-1){
        // Subscriber pose con AMCL
        poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1, &amclPoseCallback);
        bad_localization = false;
    }else if((pose_topic.find("odom")>=0 && pose_topic.find("odom")<pose_topic.size()-1) || (pose_topic.find("ground_truth")>=0 && pose_topic.find("ground_truth")<pose_topic.size()-1)){
        // Subscriber pose con odometría
        poseSub = nh.subscribe<nav_msgs::Odometry>(pose_topic, 1, &odomPoseCallback);
        bad_localization = true;
    }
    cout<<"suscrito a pose"<<endl;

    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.goal.target_pose.header.frame_id = map_topic;
    geometry_msgs::PoseStamped pathPose;

    bool save = true;

    float waitTime = 30;
    ros::Time mapTB, mapTE;
    mapTB = ros::Time::now();
    while(!grid.size() || currPose.position.x < -10000 || gridPosition.first < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.1).sleep();  // Esperar un breve periodo de tiempo
        
        mapTE = ros::Time::now();

        cout<<"ESPERANDO MAPA Y LOCALIZARME ... ("<<waitTime - (mapTE.toSec()-mapTB.toSec())<<")"<<endl;

        if(mapTE.toSec() - mapTB.toSec() > waitTime) return 0;

    }

    robotRadiusGrid = robotRadius / mapMsg.info.resolution;

    vector<vector<float>> original_grid = grid;

    //cout<<"grid despues: "<<grid.size()<<endl;

    explored_map.resize(grid.size(), vector<int>(grid[0].size(),0));
    gsx = grid.size(); gsy = grid[0].size();

    cout<<"DIMENSIONES DEL GRID: "<<gsx<<" x "<<gsy<<endl;
    cout<<"("<<currPose.position.x<<", "<<currPose.position.y<<")"<<endl;

    ros::Time beginT, endT;

    Path camino; // Para todo
    Path path2Goal, _path2Goal; vector<float> pathGradValues;
    vector<geometry_msgs::Point> path2GoalPoints;
    vector<geometry_msgs::Pose> path2GoalPoses, path2Follow;
    
    vector<vector<float>> obstGrad; // El gradiente desde los obstáculos
    vector<vector<float>> navArea; // Grid que se usa para la planificación de los movimientos del agente
    vector<vector<float>> navAreaOrig; // El original del anterior
    vector<vector<float>> navAreaOrigNorm; // El original normalizado
    vector<vector<float>> grad; // Gradiente para todo
    vector<vector<float>> navGrad; // Gradiente que se usa para planificar/navegar (navGrad = grad → camino más cercano)
    fmm_segment::Segments fmm_segments;
    vector<Poss<int>> positions; // Posiciones libres y obstáculos
    vector<geometry_msgs::Point> centroidPoints;

    beginTime = ros::Time::now();

    if(grid.size()){
        // Posición del explorador
        //currPosition = multi_util::getRobotPosition();
        //currPose = multi_util::getRobotPose();
        //gridPosition = multi_util::transformToGridCoordinates(map, currPose);
        ////cout<<grid.size()<<endl;
        //cout<<"Explorer: ("<<currPose.position.x<<", "<<currPose.position.y<<") → ("<<gridPosition.first<<", "<<gridPosition.second<<")"<<endl;
        //cout<<"OBSTACULO: "<<grid[gridPosition.first][gridPosition.second]<<endl;

        //return 0;

        // Calculo el gradiente desde la posición del agente para obtener el espacio libre en el que se va a mover
        beginT = ros::Time::now();
        FMM gr(gridPosition.first, gridPosition.second, grid);
        grad = gr.compute_gradient_();
        endT = ros::Time::now();

        // Cargarme las posiciones "no alcanzables" del grid
        positions = fix_free_space(grid, grad);

        // Gradiente de los obstáculos
        beginT = ros::Time::now();
        FMM ogr(positions[1].x, positions[1].y, grid);
        obstGrad = ogr.compute_gradient_();
        endT = ros::Time::now();

        navArea = multi_util::navigableGradient(obstGrad, (1.1*robotRadius)/mapMsg.info.resolution);
        // Cargarme las posiciones que están cerca de los obstáculos
        positions = fix_free_space(grid, navArea);

        // Goal "random" → al centro de la habitación más grande del escenario
        find_matr_max(navArea, goalGridPosition.first, goalGridPosition.second);
        goalPose = multi_util::transformFromGridCoordinates(mapMsg, goalGridPosition);

        // Gradiente desde el robot
        beginT = ros::Time::now();
        FMM cgr(gridPosition.first, gridPosition.second, navArea); // Voy a alejar al agente de los obstáculos
        navGrad = cgr.compute_gradient_();
        endT = ros::Time::now();

        // Calcular el camino hasta el goal
        path2Goal.gradient_descent_(navGrad, goalGridPosition.first, goalGridPosition.second);
        path2Goal.show();

        // Pasar camino en el grid a puntos del mapa
        path2GoalPoints = multi_util::tfGridPos2MapPos(mapMsg, path2Goal.y, path2Goal.x);
        path2GoalPoses = multi_util::convertPointsToPoses(path2GoalPoints);

        // Segmentar el entorno
        fmm_segments = fmm_segment::compute_segments(grid, navArea);
        for(int i=0; i<fmm_segments.frontier_poss.size(); i++) fmm_segments.contour_poss[i] = geometry::sort_points(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i]);
        endT = ros::Time::now();

        //// Utilizar el grid para navegar (acercamiento a los obstáculos)
        //navArea = grid;

        // Almaceno una copia del área navegable para comparar durante la navegación
        navAreaOrig = navArea;

        // Gradiente normalizado desde los obstáculos
        navAreaOrigNorm = norm_matr(navAreaOrig);

        // Centroides de los segmentos sobre el mapa
        centroidPoints = multi_util::tfGridPos2MapPos(mapMsg, fmm_segments.centroids.y, fmm_segments.centroids.x);

    }

    endTime = ros::Time::now();
    precomputationTime = endTime.toSec() - beginTime.toSec();

    cout<<"Tiempo de cálculo: "<<precomputationTime<<endl;

    vector<vector<float>> static_grid = grid;

    save = false;

    //path2Goal.pop(); // Es la posición del robot

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //                         VARIABLES PARA PUBLICAR COSAS PARA REPRESENTAR EN RViZ
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    // Para publicar el laser
    ros::Publisher laserPub = nh.advertise<nav_msgs::GridCells>("laserCells", 10);
    nav_msgs::GridCells laserGridCell;
    laserGridCell.header.seq = 0;
    laserGridCell.header.stamp = ros::Time::now();
    laserGridCell.header.frame_id = robot_frame;
    laserGridCell.cell_width = 4*mapMsg.info.resolution;
    laserGridCell.cell_height = 4*mapMsg.info.resolution;
    laserGridCell.cells = poses2Points(laserPoses);

    // Para publicar el camino planificado con FMM
    // (LO SUYO SERÍA DIRECTAMENTE PASAR ESTE CAMINO AL STACK DE NAVEGACIÓN DE ROS)
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("pathToGoal", 10);
    nav_msgs::Path rosPath;
    rosPath.header.seq = 0;
    rosPath.header.stamp = ros::Time::now();
    rosPath.header.frame_id = map_topic;
    //rosPath.poses = multi_util::transformFromGridCoordinates(map, path2Goal.y, path2Goal.x);
    rosPath.poses = multi_util::poses2PoseStamped(path2GoalPoses);

    ros::Publisher pathFollowPub = nh.advertise<nav_msgs::Path>(pathToFollow_topic, 10);
    nav_msgs::Path followPath;
    followPath.header.seq = 0;
    followPath.header.stamp = ros::Time::now();
    followPath.header.frame_id = map_topic;
    //followPath.poses = multi_util::transformFromGridCoordinates(map, path2Goal.y, path2Goal.x);
    followPath.poses = multi_util::poses2PoseStamped(path2Follow);

    // Para publicar el mapa de exploración
    ros::Publisher explMapPub = nh.advertise<nav_msgs::OccupancyGrid>("explorationMap", 10);
    nav_msgs::OccupancyGrid explorationMap;
    explorationMap.header.seq = 0;
    explorationMap.header.stamp = ros::Time::now();
    explorationMap.header.frame_id = map_topic;
    explorationMap.info = mapMsg.info; // Debería ser lo mismo
    explorationMap.data = multi_util::grid2occupancyGridData(explored_map);

    // Para publicar el mapa de los segmentos
    ros::Publisher segmMapPub = nh.advertise<nav_msgs::OccupancyGrid>("segmentsMap", 10);
    nav_msgs::OccupancyGrid segmMap;
    segmMap.header.seq = 0;
    segmMap.header.stamp = ros::Time::now();
    segmMap.header.frame_id = map_topic;
    segmMap.info = mapMsg.info; // Debería ser lo mismo
    segmMap.data = multi_util::map2occupancyGridData(fmm_segments.map);

    // Para publicar las fronteras de los segmentos
    ros::Publisher segFrontPub = nh.advertise<visualization_msgs::Marker>("segFront", 10);
    visualization_msgs::Marker segFrontPoints, segGraphPoints, segTreePoints, segBranchPoints;
    segFrontPoints.id = 0;
    segGraphPoints.id = 1;
    segTreePoints.id = 2;
    segBranchPoints.id = 3;
    segFrontPoints.header.seq = segGraphPoints.header.seq = segTreePoints.header.seq = segBranchPoints.header.seq = 0;
    segFrontPoints.header.stamp = segGraphPoints.header.stamp = segTreePoints.header.stamp = segBranchPoints.header.stamp = ros::Time::now();
    segFrontPoints.header.frame_id = segGraphPoints.header.frame_id = segTreePoints.header.frame_id = segBranchPoints.header.frame_id = map_topic;
    segFrontPoints.ns = segGraphPoints.ns = segTreePoints.ns = "segPoints";
    segFrontPoints.action = segGraphPoints.action = segTreePoints.action = segBranchPoints.action = visualization_msgs::Marker::ADD;
    segFrontPoints.pose.orientation.w = segGraphPoints.pose.orientation.w = segTreePoints.pose.orientation.w = segBranchPoints.pose.orientation.w = 1;
    segFrontPoints.type = visualization_msgs::Marker::POINTS;
    segGraphPoints.type = visualization_msgs::Marker::LINE_LIST;
    segTreePoints.type = visualization_msgs::Marker::LINE_LIST;
    segBranchPoints.type = visualization_msgs::Marker::LINE_LIST;
    segFrontPoints.scale.x = segFrontPoints.scale.y = segGraphPoints.scale.x = segGraphPoints.scale.y = mapMsg.info.resolution;
    segTreePoints.scale.x = segTreePoints.scale.y = mapMsg.info.resolution;
    segBranchPoints.scale.x = segBranchPoints.scale.y = mapMsg.info.resolution;
    segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 1.0;
    segGraphPoints.color.r = 1.0; segGraphPoints.color.a = 0.6;
    segTreePoints.color.b = 1.0; segTreePoints.color.a = 1.0;
    vector<geometry_msgs::Point> _p;
    for(int i=0; i<fmm_segments.contour_poss.size(); i++){
        _p = multi_util::tfGridPos2MapPos(mapMsg, fmm_segments.contour_poss[i].y, fmm_segments.contour_poss[i].x);
        segFrontPoints.points.insert(segFrontPoints.points.end(), _p.begin(), _p.end());
    }

    segFrontPoints.color.b = segFrontPoints.color.r = segFrontPoints.color.g = 0.8;
    //segGraphPoints.points = multi_util::buildGraphPoints(fmm_segments.centroids.y, fmm_segments.centroids.x, fmm_segments.graph, mapMsg);
    segGraphPoints.points = multi_util::linkPoints(centroidPoints, fmm_segments.graph);

    // Para publicar los objetivos del agente
    ros::Publisher targetPosPub = nh.advertise<visualization_msgs::Marker>("targetPos", 10);
    visualization_msgs::Marker targetPoints;
    targetPoints.id = 0;
    targetPoints.header.seq = 0;
    targetPoints.header.stamp = ros::Time::now();
    targetPoints.header.frame_id = map_topic;
    targetPoints.ns = "targetPoints";
    targetPoints.action = visualization_msgs::Marker::ADD;
    targetPoints.pose.orientation.w = 1;
    targetPoints.type = visualization_msgs::Marker::POINTS;
    targetPoints.scale.x = targetPoints.scale.y = mapMsg.info.resolution;
    targetPoints.color.r = 1.0; targetPoints.color.a = 1.0;

    // Para publicar los obstáculos din-amicos visibles
    ros::Publisher dynObstPub = nh.advertise<visualization_msgs::Marker>("visibleDynObst", 10);
    visualization_msgs::Marker dynObstMarker;
    dynObstMarker.id = 0;
    dynObstMarker.header.seq = 0;
    dynObstMarker.header.stamp = ros::Time::now();
    dynObstMarker.header.frame_id = map_topic;
    dynObstMarker.ns = "visibleDynObst";
    dynObstMarker.action = visualization_msgs::Marker::ADD;
    dynObstMarker.pose.orientation.w = 1;
    dynObstMarker.type = visualization_msgs::Marker::POINTS;
    dynObstMarker.scale.x = dynObstMarker.scale.y = mapMsg.info.resolution;
    dynObstMarker.color.b = 0.7; dynObstMarker.color.a = 1.0;
    //dynObstMarker.color.r = dynObstMarker.color.b = dynObstMarker.color.g = 0.2; dynObstMarker.color.a = 1.0;

    // Para publicar el árbol con los pesos
    ros::Publisher treePub = nh.advertise<visualization_msgs::MarkerArray >("tree", 10);
    visualization_msgs::MarkerArray treeLines;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    int ax, ay; // Dimensiones del agente
    Poss<int> agent, agent_; // Posiciones que ocupa el agente sobre el grid
    //ax = ay = ceil(2*0.35/mapMsg.info.resolution);
    ax = ceil(2*sX / mapMsg.info.resolution);
    ay = ceil(2*sY / mapMsg.info.resolution);
    ////cout<<ax<<", "<<ay<<endl;
    //cin.get();
    Poss<int> free_poss = positions[0]; // Todas las posiciones a ser alcanzadas

    // Variables que se usan en el método de exploración
    int vrange = (int)(laser_range/mapMsg.info.resolution); // Rango del laser (RECUERDA MIRAR EN STAGE) // ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
    vector<vector<int>> static_areas_map = fmm_segments.map;
    vector<vector<float>> static_map = grid;
    vector<Poss<int>> areas_poss = fmm_segments.poss;
    vector<vector<bool>> areas_graph = fmm_segments.graph;
    Poss<int> area_centroids = fmm_segments.centroids;
    int Nstatic_areas = area_centroids.x.size();
    exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
    int Nposs2explore = 0;
    for(int i=0; i<areas_poss.size(); i++)
        Nposs2explore += areas_poss[i].x.size();

    //cout<<laser_range<<" - "<<vrange<<" con resolución "<<mapMsg.info.resolution<<endl;
    
    // Inicializar lo que observa el agente (Sobre el grid)
    Poss<int> visible_poss, visible_obstacles, local_map_poss;
    Poss<int> visible_variation;
    Poss<int> _visible_obstacles;
    vector<geometry_msgs::Pose> visible_obstacles_poses;
    vector<geometry_msgs::Point> visible_obstacles_pos;
    Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    vector<float> visible_values;
    vector<vector<float>> world_grid; // ESTE GRID CONTIENE LOS OBSTÁCULOS DINÁMICOS
    vector<vector<bool>> obstacles_poss_map(gsx, vector<bool>(gsy, false)); // Mapa en el que se almacenan las posiciones de los obstáculos
    
    // POR AHORA ESTO, PERO LUEGO HABRÁ QUE AÑADIR LOS OBSTÁCULOS QUE NO ESTÁN EN EL MAPA ESTÁTICO
    world_grid = static_map;
    vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
    vector<float> laser_poss_values;

    // Las variables de las posiciones de los obstáculos dinámicos
    dynamic_areas::Areas areas;
    vector<vector<bool>> trajectories_map; // Mapa que almacena las trayectorías que recorren los obstáculos dinámicos
    trajectories_map = obstacles_poss_map;
    vector<Poss<int>> clusters; // Grupos de las trayectorías que han recorrido los obstáculos dinámicos
    Poss<int> obstacles_trajectories_points; // Todos los puntos que han recorrido todos los obstáculos dinámicos
    int number_of_visited_points = 0;

    // Variables para el "olvido" de las áreas dinámicas
    dynamic_areas::Areas _areas; // Las áreas dinámicas que se han construido previamente
    Poss<int> _obstacles_trajectories_points; // Todos los puntos que han recorrido todos los obstáculos dinámicos previamente
    int _number_of_visited_points = 0;
    vector<Poss<int>> _clusters; // Grupos de las trayectorías que han recorrido los obstáculos dinámicos previamente
    vector<float> estimated_distance; // Distancia estimada del recorrido del área dinámica
    vector<float> estimated_velocity; // Velocidad asignada a cad área dinámica
    vector<float> forgetting_time; // Tiempo que se espera para "olvidar" cada área dinámica
    vector<float> last_time; // Último instante en el que se ha visto cada área

    vector<geometry_msgs::Pose> cluster_poses;

    vector<vector<float>> seen_time(gsx, vector<float> (gsy, -1)); // Mapa con los instantes de tiempo en los que se ha visto cada posición del grid

    vector<int> number_of_obstacles;
    int max_obst = 0;
    
    vector<float> maxSegDist = fmm_segment::max_seg_dist(fmm_segments.centroids, fmm_segments.contour_poss);
    vector<vector<float>> distance_between_segments = fmm_segment::distance_between_segments(fmm_segments.centroids, fmm_segments.graph);
    vector<vector<float>> costs_between_segments(distance_between_segments.size(), vector<float>(distance_between_segments[0].size()));
    for(int i=0; i<costs_between_segments.size(); i++)
        for(int j=0; j<costs_between_segments[i].size(); j++)
            costs_between_segments[i][j] = distance_between_segments[i][j]/linearVel;
    vector<float> segment_obs_points = exploration::number_of_expl_points(maxSegDist, maxSegDist, vrange);
    vector<vector<int>> tree;
    vector<int> branch;
    vector<float> tree_costs, norm_tree_costs;
    vector<bool> dynamism(fmm_segments.centroids.x.size(), false);

    vector<bool> dynamism_in_stareas;

    vector<bool> pathInDynAreas;
    float d2os = 0;
    
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    path2Goal.clear(); path2GoalPoints.clear(); path2GoalPoses.clear();

    bool goalRx = false, newGoal = true, goalResult;
    float dist = INF;
    float adist = INF;

    pair<float, float> poseDist;

    Poss<int> toExplore = expl_obj.get_all_poss2explore();

    float goalTimeout = 5.0, _goalTimeout;
    ros::Time timeoutBegin, timeoutEnd;

    ros::Time obsObstBegin, obsObstEnd;
    ros::Time currTimeBegin, currTimeEnd;

    bool explore = true;
    bool replan = false;

    float rot = 0;

    vector<int> dyn_segs; // Segmentos en los que se hay obstáculos dinámicos

    //PathFollower pf("amcl_pose", "cmd_vel", maxLinearVel, maxAngularVel, 2*robotRadius, robotRadius, 0.2);
    //PathFollower pf("amcl_pose", "cmd_vel", laser_topic, maxLinearVel, maxAngularVel, 2*robotRadius, robotRadius, 1.8, 0.1, 1.0);

    vector<int> Nposs2explore_tot;
    vector<float> dist2static_obstacles, dist2dynamic_obstacles;
    vector<float> travelled_distance;
    vector<float> computationTime;

    int intrusions = 0;
    int collisions = 0;

    ros::Time computationTimeBegin, computationTimeEnd;

    beginTime = ros::Time::now();

    int cont = 0;
    while(ros::ok() && explore && max_mission_time > 0){ // Hasta que explore todo el espacoio libre

        timeoutBegin = ros::Time::now();

        currTimeBegin = ros::Time::now();
        computationTimeBegin = ros::Time::now();

        //cout<<"*************************************************************************************"<<endl;
        //cout<<"It: "<<cont<<endl;

        //cout<<"     -> El agente está en ("<<currPose.position.x<<", "<<currPose.position.y<<") | ("<<gridPosition.first<<", "<<gridPosition.second<<")"<<endl;

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //cout<<"método de exploración ..."<<endl;

        // LO que ve el explorador
        //laserPoss = fix_poss_in_map(laserPoss, grid);
        laserPoss = fix_poss_in_map(laserPoss, gsx, gsy); // eliminar las posiciones que quedan fuera del grid (por si hay un error en la localización)

        // Si estoy "mal localizado", descarto la lectura del láser
        if(bad_localization && (robotOdomPoseMsg.twist.twist.angular.z < -permissive_rotation_velocity || robotOdomPoseMsg.twist.twist.angular.z > permissive_rotation_velocity)){
            laserPoss.clear();
        }

        visible_poss = allLaserPoss(gridPosition.first, gridPosition.second, laserPoss);
        //cout<<"Posiciones del láser sibre el grid obtenidas"<<endl;

        //if(!allPossInMap(visible_poss, gsx, gsy)) continue;

        // Obtener las posiciones que ocupa el agente sobre el grid
        agent = agentPoss(gridPosition.first, gridPosition.second, ax, ay);
        //cout<<"EL AGENTE OCUPA LAS POSICIONES: "<<endl; //agent.show();

        // Añadir las posiciones del agente a las visibles
        visible_poss.append(agent);
        //cout<<"Posiciones del agente añadidas"<<endl;

        // Esto es por si estoy "mal" localizado y las posiciones del láser se salen del grid
        //visible_poss = fix_poss_in_map(visible_poss, grid);

        map_values = get_values_from_map(visible_poss, navArea); // Qué es lo que se está viendo
        //sh_vect_h(map_values,"Map_values");
        //cout<<"Obtenido lo que ve el explorador"<<endl;

        // ???????????????????????????????????????????????????????????''
        // Compruebo si estoy viendo obstáculos nuevos
        laser_poss_values = get_values_from_map(laserPoss, navArea); // Qué veo con el laser

        // -----------------------------------------------------------------------------------------------------------

        // Actualizo el "mapa de navegación"
        updateNavigableMap(navArea, navAreaOrig, laserPoss, visible_poss, ax, ay);

        // Extraer unicamente los obstáculos dinámicos
        //visible_obstacles_poses.clear();
        visible_obstacles_pos.clear();
        visible_obstacles = dynObstPos(laserPoss, laserPoses, navAreaOrig, visible_obstacles_pos);
        //visible_obstacles = dynObstPos(laserPoss, laserPoses, navAreaOrig, visible_poss, navArea, visible_obstacles_pos);
        //cout<<laserPoss.x.size()<<" - "<<visible_obstacles.x.size()<<" | "<<visible_obstacles_pos.size()<<endl;

        /*
        if(bad_localization && (robotOdomPoseMsg.twist.twist.angular.z < -0.05 || robotOdomPoseMsg.twist.twist.angular.z > 0.05)){
            laserPoss.clear();
            visible_poss.clear();
            visible_obstacles_pos.clear();
            visible_obstacles.clear();
        }
        */

        // Actualizar el listado de obstáculos
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ESTO NO VALE PARA NADA -> NO LO UTILIZO EN NINGUN SITIO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, navArea, navAreaOrig);

        // Actualizar el mapa que almacena las trayectorias recorridas por los obstáculos
        dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points);
        //cout<<number_of_visited_points<<" puntos visitados por obstáculos"<<endl;

        // Agrupar trayectorias
        //clusters = adjacency_clustering::cluster(obstacles_trajectories_points, static_map);
        clusters = clusterPoss(obstacles_trajectories_points, fmm_segments.map, fmm_segments.centroids.x.size(), dyn_segs);
        
        // Obtener las áreas dinámicas
        //areas = dynamic_areas::obtain_areas(clusters, static_map);
        areas = segs2areas(dyn_segs, fmm_segments);

        // -----------------------------------------------------------------------------------------------------------
        // Olvidar las áreas dinámicas

        if(forget_dynamic_areas){
            // Obtener las distancias "estimadas" que recorren los obstáculos dinámicos para calcular el tiempo de "olvido"
            forgetting_time.resize(areas.N, 0);
            estimated_distance.resize(areas.N, 0);
            estimated_velocity.resize(areas.N, 0);
            for(int i=0; i<areas.N; i++){
                // Obtener los segmentos que ocupan el área
                vector<int> areaInSegments = possInSegments(areas.poss[i], fmm_segments.map, fmm_segments.centroids.x.size());
                // Seleccionar la distancia máxima de los segmentos que ocupa el área
                estimated_distance[i] = 0;
                for(int j=0; j<areaInSegments.size(); j++){
                    if(estimated_distance[i] < maxSegDist[areaInSegments[j]]) estimated_distance[i] = maxSegDist[areaInSegments[j]];
                }
                // Velocidad "estimada" de movimiento de los obstáculos dentro del área
                // ESTO TENDRÍA QUE SER LA VELOCIDAD "PROPIA" DEL OBSTÁCULO VISTO O LA VELOCIDAD DEL ROBOT RECIBIDA DEL launch file
                estimated_velocity[i] = maxLinearVel/mapMsg.info.resolution; // Por ahora mi velocidad máxima
                // Tiempo de "olvido"
                forgetting_time[i] = 2*estimated_distance[i] / estimated_velocity[i];
            }

            // Actualizar el mapa de tiempos de lo visto
            assign_value(visible_poss, ros::Time::now().toSec(), seen_time);

            bool recompute_areas = false;
            last_time.resize(areas.N, 0);
            int i = 0;
            //for(int i=0; i<clusters.size(); i++){
            while(i < clusters.size()){
                last_time[i] = 0;
                for(int j=0; j<clusters[i].x.size(); j++){
                    if(last_time[i] < seen_time[clusters[i].x[j]][clusters[i].y[j]]) last_time[i] = seen_time[clusters[i].x[j]][clusters[i].y[j]];
                }
                //last_time[i] = ros::Time::now().toSec() - last_time[i];
                if(ros::Time::now().toSec() - last_time[i] >= forgetting_time[i]){ // Olvidar el área
                    // Eliminar los puntos de las variables de trajectories_map y obstacles_trajectories_points para no volver a construir las mismas áreas
                    erase_trajectories_points(clusters[i], trajectories_map, obstacles_trajectories_points, number_of_visited_points);
                    for(int j=0; j<clusters[i].x.size(); j++)
                        navArea[clusters[i].x[j]][clusters[i].y[j]] = navAreaOrig[clusters[i].x[j]][clusters[i].y[j]];
                    clusters.erase(clusters.begin() + i);
                    dyn_segs.erase(dyn_segs.begin() + i);
                    // Eliminar el área
                    //areas.erase_area(i);
                    //cout<<"área "<<i<<" borrada"<<endl;
                    recompute_areas = true;
                }else
                    i++;
            }

            // Volver a obtener las áreas dinámicas
            //if(recompute_areas) areas = dynamic_areas::obtain_areas(clusters, static_map);
            if(recompute_areas) areas = segs2areas(dyn_segs, fmm_segments);
        }

        //save_matr("seen_time.txt", seen_time);

        //sh_vect_h(estimated_distance, "d");
        //sh_vect_h(estimated_velocity, "v");
        //sh_vect_h(forgetting_time, "forgetting_time");
        //sh_vect_h(last_time, "last_time");



        // -----------------------------------------------------------------------------------------------------------

        // ACTUALIZAR LAS VARIABLES DE LAS ÁREAS DINÁMICAS
        number_of_obstacles = number_of_obstacles_in_segments_zs(obstacles_list, areas.map, areas.N);
        max_obst = 0;
        for(int i=0; i<areas.N; i++){
            if(max_obst < number_of_obstacles[i]) max_obst = number_of_obstacles[i];
            number_of_obstacles[i] = number_of_obstacles[i] ? number_of_obstacles[i] : 1; // Para que las zonas que no se ven tengan menos prioridad que espacio vacío
        }

        //cout<<obstacles_list.x.size()<<" | "<<visible_obstacles.x.size()<<" listado de obstáculos"<<endl;
        //cout<<clusters.size()<<" - "<<areas.N<<endl;
        //sh_vect_h(number_of_obstacles,"O/A");

        //int nax, nay;
        //float naMax;
        //naMax = find_matr_max_nf(navArea, nax, nay);
        //cout<<"Antes ("<<nax<<", "<<nay<<"): "<<naMax<<endl;

        // -----------------------------------------------------------------------------------------------------------

        // Insertar la información de las áreas dinámicas en el mapa para planificar/navegar
        //navigation_grid = dynamic_areas::navigable_grid(static_map, free_poss, obstacles_list, areas_poss, number_of_obstacles);
        //navArea = dynamic_areas::navigable_grid_occup_thres(navAreaOrig, free_poss, obstacles_list, areas_poss, number_of_obstacles, occup_thres, static_areas_map[gridPosition.first][gridPosition.second]);
        //navArea = dynamic_areas::navigable_grid_occup_thres(navAreaOrig, free_poss, obstacles_list, areas.poss, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        //navArea = navigable_norm_grad_occup_thres(navAreaOrigNorm, free_poss, obstacles_list, areas.poss, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        //navArea = navigable_grad_occup_thres(navAreaOrig, free_poss, obstacles_list, areas.poss, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        navArea = navigable_grad_occup_thres(navAreaOrig, free_poss, obstacles_list, areas.poss, areas.contours_edges, 2*ax, 2*ay, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        //navArea = navigable_grad_occup_thres(navAreaOrig, free_poss, visible_obstacles, areas.poss, areas.contours_edges, 2*ax, 2*ay, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        // Añadir tambien los obstáculos ("VISIBLES") EN EL GRID
        for(int i=0; i<visible_obstacles.x.size(); i++){
            navArea[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;
            if(!dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]]) dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]] = true;
        }

        save_matr("navArea.txt", navArea);
        save_matr("dynamicAreas.txt", areas.map);

        //cout<<"ESTOY EN "<<areas.map[gridPosition.first][gridPosition.second]<<endl;
        //cout<<"ESTOY EN "<<fmm_segments.map[gridPosition.first][gridPosition.second]<<endl;
        //if(fmm_segments.map[gridPosition.first][gridPosition.second] >= 0 && dynamism[fmm_segments.map[gridPosition.first][gridPosition.second]]){
        //    cout<<"ESTOY DENTRO DE UN SEGMENTO DONDE HAY OBSTÁCULOS DINÁMICOS"<<endl;
        //    path2Goal = runawayPath(gridPosition.first, gridPosition.second, navArea, fmm_segments.map, dynamism, fmm_segments.poss);
        //}else{

            // ???????????????????????????????????????????????????????????''

            // Actualizar las variables de exploración (mapas basicamente)
            expl_obj.update_all(visible_poss, map_values);
            //cout<<"Actualizado en el método de exploración"<<endl;

            // SELECCIÓN DE LOS NUEVOS OBJETIVOS
            // Reiniciar los objetivos
            expl_obj.update_targets();
            //cout<<"Objetivos reiniciados"<<endl;

            // Almacenar el camino antiguo
            _path2Goal = path2Goal;

            //cout<<"camino: "<<path2Goal.x.size()<<endl;
            //cout<<"Donde está el agente: ("<<gridPosition.first<<", "<<gridPosition.second<<"): "<<navArea[gridPosition.first][gridPosition.second]<<endl;

            // Calcular nuevo objetivo (partición + conjunto de puntos + goal) y el camino hacia él
            //expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, distance_between_segments, segment_obs_points, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs);
            expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, distance_between_segments, segment_obs_points, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs, branch);
            //cout<<"Objetivos y camino obtenidos"<<endl;

            // ???????????????????????????????????????????????????????????''
            // Comprobar si no es posible obtener camino, por los obstáculos dinámicos
            if(!path2Goal.tam && navArea[gridPosition.first][gridPosition.second]){
                cout<<"ACTUALIZO EL GRAFO"<<endl;
                vector<bool> dynamism_in_stareas; // Dinamismo en áreas dinámicas
                dynamism_in_stareas = dynamic_areas::in_area(areas.poss, static_areas_map, Nstatic_areas);
                //sh_vect_h(dynamism_in_stareas, "D/A");
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
                //cout<<"traversability"<<endl;
                //expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph,distance_between_segments, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs);
                //cout<<"hecho"<<endl;

                //cout<<"donde estoy: "<<navArea[gridPosition.first][gridPosition.second]<<endl;
            }
            // ???????????????????????????????????????????????????????????''
        //}

        //explore = expl_obj.get_target_area() >= 0;
        //toExplore = expl_obj.get_all_poss2explore();
        //explore = toExplore.x.size() > 0;

        //cout<<"explorar "<<explore<<endl;
        //cout<<"camino: "<<path2Goal.x.size()<<endl;

        //cout<<"HE seleccionado segmento "<<expl_obj.get_target_area()<<", conjunto "<<expl_obj.get_target_set()<<endl;

        replan = false;
        if(path2Goal.x.size()){

            // Hay un obstáculo al final del camino
            agent_ = agentPoss(expl_obj.getGoalx(), expl_obj.getGoaly(), ax, ay);
            vector<float> agent_in_goal_values = get_values_from_map(agent_, navArea);

            for(int i=0; i<agent_in_goal_values.size(); i++)
                if(agent_in_goal_values[i] == 0){
                    //sh_vect_h(agent_in_goal_values,"agent_in_goal_values");
                    //cout<<"¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡"<<endl;
                    //cout<<"        ABORTA, OBSTÁCULO EN EL GOAL     "<<endl;
                    //cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                    replan = true;
                    break;
                }

            if(replan){
                cout<<"Replanificando ..."<<endl;
                vector<bool> dynamism_in_stareas; // Dinamismo en áreas dinámicas
                dynamism_in_stareas = dynamic_areas::in_area(areas.poss, static_areas_map, Nstatic_areas);
                //cout<<"Detectados nuevos obstáculos"<<endl;
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
                //cout<<"Posiciones no alcanzables descartadas"<<endl;
                //expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs);
                //cout<<"Nuevos objetivos y camino obtenidos"<<endl;
            }

            // Pasar camino en el grid a puntos del mapa
            path2GoalPoints = multi_util::tfGridPos2MapPos(mapMsg, path2Goal.y, path2Goal.x);
            //showPoints(path2GoalPoints);
            path2GoalPoses = multi_util::convertPointsToPoses(path2GoalPoints);
            //showPoses(path2GoalPoses);

            // Obtener la distancia del camino a los obstáculos más cercanos
            pathGradValues.resize(path2Goal.x.size(), 0);
            path2Follow = path2GoalPoses;
            float maxv = -1;
            for(int i=0; i<path2Goal.x.size(); i++){
                pathGradValues[i] = navArea[path2Goal.x[i]][path2Goal.y[i]];
                if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
            }

            // Incluir las distancias de los obstáculos dinámicos en el camino
            float _d;
            for(int i=0; i<path2Goal.x.size(); i++){
                for(int j=0; j<visible_obstacles.x.size(); j++){
                    _d = hypot(visible_obstacles.x[j] - path2Goal.x[i], visible_obstacles.y[j] - path2Goal.y[i]);
                    if(pathGradValues[i] < _d){
                        pathGradValues[i] = _d;
                    }
                    if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
                }
            }

            // Normalizar los valores de las distancias del camino
            if(maxv)
                for(int i=0; i<path2Goal.x.size(); i++){
                    //pathGradValues[i] /= maxv;
                    //path2Follow[i].position.z = pathGradValues[i]; // Distancia sobre el grid
                    path2Follow[i].position.z = pathGradValues[i] * mapMsg.info.resolution; // Distancia sobre el mapa
                }

            //cout<<"Distancia máxima: "<<maxv * mapMsg.info.resolution<<" - "<<robotRadius<<endl;
            
        }

        //cout<<"replanificado"<<endl;

        explore = expl_obj.remaining_poss2explore();

        // Comprobar si he llegado al goal y no veo lo que esperaba ver
        //if((expl_obj.getGoalx() == gridPosition.first && expl_obj.getGoaly() == gridPosition.second)){ // Estoy en el objetivo
        if(path2Goal.x.size() <= 2*robotRadiusGrid){ // Estoy en el objetivo
            // Comprobar cuanto tiempo llevo en el punto
            timeoutEnd = ros::Time::now();
            _goalTimeout -= (timeoutEnd - timeoutBegin).toSec();
            if(_goalTimeout <= 0){
                //cout<<"TIEMPO DE ESPERA EXPIRADO"<<endl;
                //cout<<"quito los puntos del área "<<expl_obj.get_target_area()<<" de la exploración"<<endl;
                // Quitar lo puntos del área de la exploración
                expl_obj.erase_area(expl_obj.get_target_area());
            }
        }else{
            _goalTimeout = goalTimeout;
        }

        //cout<<"goal timeout comprobado"<<endl;

        computationTimeEnd = ros::Time::now();

        // Obtener la distancia del robot al obstáculo más cercano
        float md = visible_obstacles.minDistToPoint(gridPosition.first, gridPosition.second);
        if(md < robotRadiusGrid){
            intrusions++;
            //cout<<"COLISIONANDO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            //cout<<"POSE: ("<<currPose.position.x<<", "<<currPose.position.y<<") - ("<<currPose.orientation.x<<", "<<currPose.orientation.y<<", "<<currPose.orientation.z<<", "<<currPose.orientation.w<<")"<<endl;
            poseDist = multi_util::distPose2Path(currPose, totalPath, 10);
            //cout<<"DISTANCIA: "<<poseDist.first<<" , "<<poseDist.second<<endl;
            if(poseDist.first + poseDist.second){
                cout<<"COLISION"<<endl;
                explore = false;
                collisions++;
            }
        }
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        //cout<<"publico topics ..."<<endl;

        // PUBLICAR LOS TOPICS DE LAS COSAS QUE QUIERO REPRESENTAR
        // Publicar las celdas para representar sobre el mapa en RViz
        laserGridCell.cells = poses2Points(laserPoses); // Actualizar lo que voy a publicar
        laserGridCell.header.stamp = ros::Time::now();
        laserPub.publish(laserGridCell); // Publico

        // Publicar el camino que sigue el robot
        rosPath.poses = multi_util::poses2PoseStamped(path2GoalPoses);
        rosPath.header.seq = cont;
        rosPath.header.frame_id = map_topic;
        rosPath.header.stamp = ros::Time::now();
        pathPub.publish(rosPath); // Publico

        //cout<<path2Goal.x.size()<<" - "<<path2Follow.size()<<endl;

        followPath.poses = multi_util::poses2PoseStamped(path2Follow);
        followPath.header.seq = cont;
        followPath.header.frame_id = map_topic;
        followPath.header.stamp = ros::Time::now();
        pathFollowPub.publish(followPath); // Publico

        // Publicar el mapa de exploración
        //explorationMap.data = multi_util::grid2occupancyGridData(explored_map);
        //explorationMap.data = multi_util::map2occupancyGridData(navArea);
        explorationMap.data = multi_util::map2occupancyGridData(expl_obj.get_seen_map());
        //explorationMap.data = multi_util::map2occupancyGridData(areas.map); // mapa dinámico
        explMapPub.publish(explorationMap); // Publico

        // Publicar los segmentos
        segmMapPub.publish(segmMap); // Publico

        // Publicar los puntos de los segmentos
        //segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 0.25;
        segFrontPub.publish(segFrontPoints);
        segGraphPoints.color.r = 1.0; segGraphPoints.color.a = 0.15;
        segFrontPub.publish(segGraphPoints);

        // Publicar los centroides de los segmentos con el grafo del entorno
        //segGraphPoints.points = multi_util::buildGraphPoints(fmm_segments.centroids.y, fmm_segments.centroids.x, areas_graph, mapMsg); // Representar el grafo
        //segGraphPoints.points = multi_util::buildTreePoints(fmm_segments.centroids.y, fmm_segments.centroids.x, tree, mapMsg); // Representar árbol
        //segTreePoints.points = multi_util::buildTreePoints(fmm_segments.centroids.y, fmm_segments.centroids.x, tree, mapMsg); // Representar árbol
        segTreePoints.points = multi_util::linkPoints(centroidPoints, tree); // Representar árbol
        segTreePoints.color.b = 1.0; segTreePoints.color.a = 0.25;
        segFrontPub.publish(segTreePoints);

        segBranchPoints.points = multi_util::linkPoints(centroidPoints, branch); // Representar árbol
        segBranchPoints.color.b = 1.0; segBranchPoints.color.a = 1;
        segFrontPub.publish(segBranchPoints);

        // Publicar las posiciones objetivo del agente
        if(expl_obj.get_target_area() >= 0){
            Poss<int> _target = expl_obj.get_poss2explore()[expl_obj.get_target_area()];
            targetPoints.points = multi_util::tfGridPos2MapPos(mapMsg, _target.y, _target.x);
            //targetPoints.color.r = 1.0; targetPoints.color.a = 0.25;
            targetPosPub.publish(targetPoints);
        }

        // Publicar las ramas del árbol
        //norm_tree_costs = normalize_tree_costs(tree, tree_costs);
        //sh_vect_h(norm_tree_costs);
        //treeLines = setTreeLines(centroidPoints, tree, norm_tree_costs, map_topic, "treeLines");
        //treePub.publish(treeLines);

        // Publicar los obstáculos dinámicos vistos por el agente
        dynObstMarker.points = visible_obstacles_pos;
        dynObstMarker.points = poss2Points(areas.contours_edges, mapMsg); // Contornos de las áreas dinámicas
        //dynObstMarker.points = multi_util::tfGridPos2MapPos(mapMsg, obstacles_list.y, obstacles_list.x);
        dynObstPub.publish(dynObstMarker);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        cont++;

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loopRate.sleep();

        //if(_visible_obstacles == visible_obstacles)
            
        if(_visible_obstacles != visible_obstacles){
            _robotOdomPoseMsg = robotOdomPoseMsg;
            rot = robotOdomPoseMsg.twist.twist.angular.z;
        }

        _visible_obstacles = visible_obstacles;
        
        currTimeEnd = ros::Time::now();

        max_mission_time -= (currTimeEnd.toSec() - currTimeBegin.toSec());

        std::system("clear");

        // Almacenar variables de los resultados
        Nposs2explore_tot.push_back(expl_obj.Nremaining_poss2explore());
        dist2dynamic_obstacles.push_back(distance2closest_obstacle(gridPosition.first, gridPosition.second, visible_obstacles) * mapMsg.info.resolution);
        dist2static_obstacles.push_back(obstGrad[gridPosition.first][gridPosition.second] * mapMsg.info.resolution);
        travelled_distance.push_back(multi_util::computePathDistance(totalPath));

        computationTime.push_back(computationTimeEnd.toSec() - computationTimeBegin.toSec());
        
        cout<<"Distancia recorrida: "<<multi_util::computePathDistance(totalPath)<<endl;
        cout<<"Distancia recorrida: "<<travelled_distance[travelled_distance.size() - 1]<<endl;
        cout<<"Puntos por explorar: "<<Nposs2explore_tot[Nposs2explore_tot.size() - 1]<<" | "<<Nposs2explore<<endl;
        cout<<"Distancia al obstáculo: "<<dist2dynamic_obstacles[dist2dynamic_obstacles.size() - 1]<<" | "<<dist2static_obstacles[dist2static_obstacles.size() - 1]<<endl;
        cout<<"TIEMPO RESTANTE DE MISSIÓN: "<<max_mission_time<<endl;
        cout<<"Tiempo de cálculo: "<<computationTime[computationTime.size()-1]<<endl;
        

    }

    endTime = ros::Time::now();
    missionTime = endTime.toSec() - beginTime.toSec();

    float explored = exploration::explored_area(expl_obj.get_real_seen_map(), navAreaOrig);

    cout<<"Distancia recorrida: "<<multi_util::computePathDistance(totalPath)<<endl;
    cout<<"Tiempo de cálculo: "<<precomputationTime<<endl;
    cout<<"Tiempo de misión: "<<missionTime<<endl;
    cout<<"Explorado: "<<explored<<endl;

    // Publicar camino vacío para que el path follower pare
    path2Follow.clear();
    followPath.poses = multi_util::poses2PoseStamped(path2Follow);
    followPath.header.seq = cont;
    followPath.header.frame_id = map_topic;
    followPath.header.stamp = ros::Time::now();
    pathFollowPub.publish(followPath); // Publico

    cout<<"!!!!! WE DID IT BABY. BRING THE RED PANTIES !!!!!"<<endl;

    // Guardar los resultados en fichero
    vector<float> res_vect = {multi_util::computePathDistance(totalPath), precomputationTime, missionTime, explored, (float)intrusions, (float)collisions};
    vector<float> size_vect = {(float)sX, (float)sY, (float)robotRadius, (float)ax, (float)ay};
    save_vect(result_filename.c_str(), res_vect);
    save_vect_(result_filename.c_str(), size_vect, "size");
    save_vect_(result_filename.c_str(), Nposs2explore_tot, "poss2explore");
    save_vect_(result_filename.c_str(), travelled_distance, "distance");
    save_vect_(result_filename.c_str(), dist2dynamic_obstacles, "dist2dynObst");
    save_vect_(result_filename.c_str(), dist2static_obstacles, "dist2stObst");
    save_vect_(result_filename.c_str(), computationTime, "computationTime");

    //ros::shutdown(); // "Cerrar" este nodo

    // He terminado, "cierro" todos los nodos
    std::system("rosnode kill --all");
    std::system("killall -9 rosmaster");
    std::system("killall -9 roscore");

    // // //std::system("roslaunch multi_dynamic scenario10_loop.launch");

    return 0;
}
