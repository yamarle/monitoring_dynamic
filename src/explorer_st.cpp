
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

#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>

// TODAS LAS VARIABLES GLOBALES QUE VA A USAR EL EXPLORADOR


// PÁRAMETROS DE ENTRADA
double sX, sY; // Tamaño del robot
string map_topic, pose_topic, laser_topic, pathToFollow_topic;
string robot_frame;
double laser_range;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // Para enviar los objetivos al robot

// Posiciones que usa el robot
pair<int, int> gridPosition, goalGridPosition, waypointGrid;
geometry_msgs::PoseWithCovarianceStamped realPose;
geometry_msgs::Pose currPose, goalPose, waypointPose;
geometry_msgs::Point explPoint, goalPoint, wayPoint;

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
float maxLinearVel = 0.65;
float maxAngularVel = 1.0;
float robotRadius = 0.5;

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

Poss<int> getExploredPoss()
{
    Poss<int> res;
    for(int i=0; i<explored_map.size(); i++)
        for(int j=0; j<explored_map[i].size(); j++)
            if(explored_map[i][j]) res.push(i,j);
    return res;
}

Poss<int> laserFix(Poss<int> laserPoss, int sx, int sy)
{
    for(int i=0; i<laserPoss.x.size(); i++)
        if(laserPoss.x[i] < 0 || laserPoss.x[i] > sx || laserPoss.x[i] < 0 || laserPoss.x[i] > sx){
            laserPoss.clear();
            return laserPoss;
        }
    return laserPoss;
}

vector<geometry_msgs::Point> laserTf(vector<geometry_msgs::Point> points, pair<double, double> dp)
{
    for(int i=0; i<points.size(); i++){
        points[i].x += dp.first; points[i].y += dp.second;
    }
    return points;
}

vector<geometry_msgs::Point> laserFix(vector<geometry_msgs::Point> points, double sx, double sy)
{
    for(int i=0; i<points.size(); i++)
        if(points[i].x < 0 || points[i].x > sx || points[i].y < 0 || points[i].y > sx){
            points.clear();
            return points;
        }
    return points;
}

void showPoints(vector<geometry_msgs::Point> points)
{
    for(int i=0; i<points.size(); i++)
        cout<<points[i].x<<", "<<points[i].y<<endl;
}

bool sendGoal(double x, double y)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    //goal.target_pose.header.frame_id = robot_frame; // Con respecto al robot
    goal.target_pose.header.frame_id = map_topic; // Coordenada global
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;
    
    ROS_INFO("Sending goal: (%f, %f)", x, y);
    ac.sendGoal(goal);

    return ac.waitForResult();
}

bool sendGoal(geometry_msgs::Pose goalPose)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    //goal.target_pose.header.frame_id = robot_frame; // Con respecto al robot
    goal.target_pose.header.frame_id = map_topic; // Coordenada global
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose = goalPose;

    
    ROS_INFO("Sending goal: (%f, %f, %f)", goalPose.position.x, goalPose.position.y, goalPose.orientation.w);
    ac.sendGoal(goal);

    //return ac.waitForResult();
    return true;
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

Poss<int> possLoss(int x, int y, Poss<int> poss)
{
    Poss<int> res;
    for(int i=0; i<poss.x.size(); i++)
        if(!bresenham::exists_wall(grid, x, y, poss.x[i], poss.y[i])) res.push(poss.x[i], poss.y[i]);
    return res;   
}

vector<bool> checkPossLoS(int x, int y, Poss<int> poss)
{
    vector<bool> res;
    for(int i=0; i<poss.x.size(); i++)
        res.push_back(!bresenham::exists_wall(grid, x, y, poss.x[i], poss.y[i]));
    return res;   
}

int selectWaypoint(int x, int y, Path path, int pathStep)
{
    for(int i=0; i<pathStep; i++)
        if(bresenham::exists_wall(grid, x, y, path.x[i], path.y[i])) return i-1;
    return 0;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{

    laserPoses.clear();
    laserGridPoses.clear();
    laserGridPos.clear();
    laserPoss.clear();

    tf::TransformListener listener;
    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped laserPoseStamped;
    laserPoseStamped.header.frame_id = robot_frame;
    //laserPoseStamped.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped transformedPose;
    

    float angle;
    geometry_msgs::Pose pose;
    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < laser_scan->ranges.size(); ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        angle = laser_scan->angle_min + i * laser_scan->angle_increment;

        // Verificar si la distancia medida está dentro del rango válido
        if (laser_scan->ranges[i] >= laser_scan->range_min && laser_scan->ranges[i] <= laser_scan->range_max){
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = laser_scan->ranges[i] * cos(angle);
            pose.position.y = laser_scan->ranges[i] * sin(angle);
            pose.position.z = 0.0;

            // Creo el quaternion
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            laserPoses.push_back(pose);

            // *********************************************
            // Transformar las posiciones del láser a las coordenadas globales del mapa
            laserPoseStamped.pose = pose;
            laserPoseStamped.header.stamp = ros::Time::now();
            //laserPoseStamped.header.stamp = laser_scan->header.stamp + ros::Duration().fromSec(i*laser_scan->scan_time);
            try{
                listener.transformPose(map_topic, laserPoseStamped, transformedPose);
                laserGridPoses.push_back(transformedPose.pose);
                laserGridPos.push_back(multi_util::transform2GridCoordinates(mapMsg, transformedPose.pose.position.x, transformedPose.pose.position.y));
                laserPoss.push(laserGridPos[laserGridPos.size()-1].y, laserGridPos[laserGridPos.size()-1].x);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("Error al transformar la pose del láser: %s", ex.what());
            }
            // *********************************************
        }
    }
    //cout<<"ELTIEMPO: "<<ros::Time::now()<<endl;

    //laserGridPos = multi_util::transform2GridCoordinates(mapMsg, laserGridPoses);
    //laserPoss = points2Poss(laserGridPos);
    
}


void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
    //ros::Duration(1.0).sleep();
    //cout<<"SUSCRITO CON AMCL"<<endl;
    currPose = robotPoseMsg->pose.pose;
    gridPosition = multi_util::transformToGridCoordinates(mapMsg, currPose);
}

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    //ros::Duration(1.0).sleep();
    //cout<<"SUSCRITO CON odom"<<endl;
    currPose = robotPoseMsg->pose.pose;
    gridPosition = multi_util::transformToGridCoordinates(mapMsg, currPose);
}

void robotSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg, const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    // Obtener la lectura del laser
    laserCallback(laser_msg);

    // Obtener la pose del robot
    amclPoseCallback(pose_msg);
    
    //cout<<"---------------------------------->   POSE + LASER: "<<pose_msg->header.stamp<<" - "<<laser_msg->header.stamp<<endl;
}

//****************************************************************************************************************
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

// ***************************************************************************************************************************************

// FUNCIONES PARA EL CÁLCULO DE VELOCIDADES

void computeVelocities(const geometry_msgs::Pose currentPose, const geometry_msgs::Pose goalPose, double linearSpeed, double angularSpeed, double& linearVel, double& angularVel)
{
    // Cálculo del ángulo hacia el punto objetivo
    double dx = goalPose.position.x - currentPose.position.x;
    double dy = goalPose.position.y - currentPose.position.y;
    double targetAngle = atan2(dy, dx);

    // Cálculo del ángulo de desviación
    double angleDiff = targetAngle - tf::getYaw(currentPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular
    linearVel = linearSpeed;
    angularVel = angularSpeed * angleDiff;
}


// Función para calcular las velocidades lineal y angular usando el algoritmo de Ventana Dinámica
void computeVelocitiesDWA(const geometry_msgs::Pose currentPose, const geometry_msgs::Pose goalPose, const sensor_msgs::LaserScan laserData, double max_linear_speed , double max_angular_speed, double robot_radius, double& linearVel, double& angularVel)
{
    // Parámetros del algoritmo
    const double ANGULAR_RESOLUTION = 0.1;  // Resolución angular en radianes
    const double MAX_OBSTACLE_DISTANCE = 1.0;  // Distancia máxima para considerar un obstáculo en metros

    // Inicialización de las velocidades
    linearVel = 0.0;
    angularVel = 0.0;

    // Cálculo del ángulo hacia el punto objetivo
    double dx = goalPose.position.x - currentPose.position.x;
    double dy = goalPose.position.y - currentPose.position.y;
    double targetAngle = atan2(dy, dx);

    // Ajuste del ángulo hacia el punto objetivo en el rango [-pi, pi]
    if (targetAngle > M_PI)
        targetAngle -= 2.0 * M_PI;
    else if (targetAngle < -M_PI)
        targetAngle += 2.0 * M_PI;

    // Búsqueda de la mejor velocidad lineal y angular dentro de la ventana dinámica
    double bestLinearVel = 0.0;
    double bestAngularVel = 0.0;
    double minCost = std::numeric_limits<double>::max();

    for (double angularVel = -max_angular_speed; angularVel <= max_angular_speed; angularVel += ANGULAR_RESOLUTION)
    {
        // Cálculo de la velocidad lineal correspondiente al radio de curvatura
        double linearVel = std::min(max_linear_speed, std::abs(angularVel) * robot_radius);

        // Cálculo del costo de la velocidad actual
        double cost = 0.0;

        // Simulación de movimiento para calcular el costo
        for (double t = 0.0; t <= 1.0; t += 0.1)
        {
            // Posición y orientación simuladas
            double simX = currentPose.position.x + linearVel * cos(currentPose.orientation.z + angularVel * t);
            double simY = currentPose.position.y + linearVel * sin(currentPose.orientation.z + angularVel * t);

            // Cálculo de la distancia al objetivo
            double distanceToGoal = std::hypot(simX - goalPose.position.x, simY - goalPose.position.y);

            // Cálculo del costo como una combinación de distancia al objetivo y proximidad a obstáculos
            double obstacleCost = 0.0;
            for (size_t i = 0; i < laserData.ranges.size(); ++i)
            {
                double angle = laserData.angle_min + i * laserData.angle_increment;
                double obstacleDistance = laserData.ranges[i];

                // Ignorar lecturas de láser inválidas o demasiado distantes
                if (std::isnan(obstacleDistance) || obstacleDistance > MAX_OBSTACLE_DISTANCE)
                    continue;

                // Calcular proximidad a obstáculos y sumar al costo
                double obstacleAngle = angle + currentPose.orientation.z + angularVel * t;
                double obstacleX = currentPose.position.x + obstacleDistance * cos(obstacleAngle);
                double obstacleY = currentPose.position.y + obstacleDistance * sin(obstacleAngle);
                double distanceToObstacle = std::hypot(obstacleX - simX, obstacleY - simY);
                obstacleCost += 1.0 / distanceToObstacle;
            }

            // Costo total de la velocidad actual
            double totalCost = distanceToGoal + obstacleCost;

            // Actualizar la mejor velocidad y costo
            if (totalCost < minCost)
            {
                minCost = totalCost;
                bestLinearVel = linearVel;
                bestAngularVel = angularVel;
            }
        }
    }

    // Asignación de las velocidades resultantes
    linearVel = bestLinearVel;
    angularVel = bestAngularVel;
}

// ***************************************************************************************************************************************

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer");
    ros::NodeHandle nh;

    // Get tf_prefix from the parameter server
    string tf_prefix;
    nh.getParam("tf_prefix", tf_prefix);

    ros::Rate loopRate(500);

    if(argc == 9){
        map_topic = argv[1];
        pose_topic = argv[2];
        robot_frame = argv[3];
        laser_topic = argv[4];
        pathToFollow_topic = argv[5];
        sX = atof(argv[6]);
        sY = atof(argv[7]);
        laser_range = atof(argv[8]);
    }

    cout<<argc<<" argumentos"<<endl;
    cout<<"map topic:   "<<map_topic<<endl;
    cout<<"pose topic:  "<<pose_topic<<endl;
    cout<<"robot frame: "<<robot_frame<<endl;
    cout<<"laser topic: "<<laser_topic<<endl;
    cout<<"path topic:  "<<pathToFollow_topic<<endl;
    cout<<"tamaño:      "<<sX<<" x "<<sY<<endl;
    cout<<"rango laser: "<<laser_range<<endl;
    //return 0;

    robotRadius = hypot(sX, sY);

    currPose.position.x = -INF;

    // Me suscribo una vez
    
    //ros::Subscriber occGridSub = nh.subscribe(map_topic, 1, &occupancyMapCallback);
    //ros::Subscriber laserSub = nh.subscribe(laser_topic, 1, &laserPosCallback);
    //nav_msgs::OccupancyGrid map;
    vector<vector<int>> occGrid;
    Poss<int> obst;
    //ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::_occupancyGridCallback, _1, boost::ref(occGrid)));
    //ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::occupancyGridCallback, _1, boost::ref(map), boost::ref(occGrid)));
    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::gridCallback, _1, boost::ref(mapMsg), boost::ref(occGrid), boost::ref(grid)));
    
    //ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, boost::bind(multi_util::laserPosCallback, _1, boost::ref(laserPos)));
    // → ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, boost::bind(multi_util::laserPosesCallback, _1, boost::ref(laserPoses)));
    
    /*
    ros::Subscriber laser_Sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &laserCallback);
    ros::Subscriber pose_Sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1, &amclPoseCallback);
    */

    // Subscriber laser
    LaserScanToPointCloud lstopc(nh, map_topic);
    ros::Subscriber poseSub;
    if(pose_topic.find("amcl")>=0 && pose_topic.find("amcl")<pose_topic.size()-1){
        // Subscriber pose con AMCL
        poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1, &amclPoseCallback);
    }else if((pose_topic.find("odom")>=0 && pose_topic.find("odom")<pose_topic.size()-1) || (pose_topic.find("ground_truth")>=0 && pose_topic.find("ground_truth")<pose_topic.size()-1)){
        // Subscriber pose con odometría
        poseSub = nh.subscribe<nav_msgs::Odometry>(pose_topic, 1, &odomPoseCallback);
    }
    cout<<"suscrito a pose"<<endl;

    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.goal.target_pose.header.frame_id = map_topic;
    geometry_msgs::PoseStamped pathPose;

    /*
    // Definir los suscriptores para los mensajes de pose y láser scan
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, laser_topic, 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, pose_topic, 1);

    // Definir la política de sincronización (en este caso, sincronización aproximada en base al tiempo)
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> SyncPolicy;
    //message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, laser_sub); // El número 10 es el tamaño del buffer
    message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> sync(pose_sub, laser_sub, 1);

    // Establecer la función de callback para la sincronización de los mensajes
    sync.registerCallback(boost::bind(&robotSyncCallback, _1, _2));
    */

    bool save = true;


    while(!grid.size() || currPose.position.x < -10000 || gridPosition.first < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.1).sleep();  // Esperar un breve periodo de tiempo
        //cout<<"ESPERANDO MAPA Y LOCALIZARME ..." <<endl;
    }

    //cout<<"grid despues: "<<grid.size()<<endl;


    explored_map.resize(grid.size(), vector<int>(grid[0].size(),0));
    gsx = grid.size(); gsy = grid[0].size();

    ros::Time beginT, endT;

    Path camino; // Para todo
    Path path2Goal, _path2Goal; vector<float> pathGradValues;
    vector<geometry_msgs::Point> path2GoalPoints;
    vector<geometry_msgs::Pose> path2GoalPoses, path2Follow;
    
    vector<vector<float>> obstGrad; // El gradiente desde los obstáculos
    vector<vector<float>> navArea; // Grid que se usa para la planificación de los movimientos del agente
    vector<vector<float>> navAreaOrig; // El original del anterior
    vector<vector<float>> grad; // Gradiente para todo
    vector<vector<float>> navGrad; // Gradiente que se usa para planificar/navegar (navGrad = grad → camino más cercano)
    fmm_segment::Segments fmm_segments;
    vector<Poss<int>> positions; // Posiciones libres y obstáculos

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

        //navArea = grid;

        // Almaceno una copia del área navegable para comparar durante la navegación
        navAreaOrig = navArea;



    }

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
    visualization_msgs::Marker segFrontPoints, segCentrPoints;
    segFrontPoints.id = 0;
    segCentrPoints.id = 1;
    segFrontPoints.header.seq = segCentrPoints.header.seq = 0;
    segFrontPoints.header.stamp = segCentrPoints.header.stamp = ros::Time::now();
    segFrontPoints.header.frame_id = segCentrPoints.header.frame_id = map_topic;
    segFrontPoints.ns = segCentrPoints.ns = "segPoints";
    segFrontPoints.action = segCentrPoints.action = visualization_msgs::Marker::ADD;
    segFrontPoints.pose.orientation.w = segCentrPoints.pose.orientation.w = 1;
    segFrontPoints.type = visualization_msgs::Marker::POINTS;
    segCentrPoints.type = visualization_msgs::Marker::LINE_LIST;
    segFrontPoints.scale.x = segFrontPoints.scale.y = segCentrPoints.scale.x = segCentrPoints.scale.y = mapMsg.info.resolution;
    segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 1.0;
    segCentrPoints.color.r = 1.0; segCentrPoints.color.a = 1.0;
    vector<geometry_msgs::Point> _p;
    for(int i=0; i<fmm_segments.contour_poss.size(); i++){
        _p = multi_util::tfGridPos2MapPos(mapMsg, fmm_segments.contour_poss[i].y, fmm_segments.contour_poss[i].x);
        segFrontPoints.points.insert(segFrontPoints.points.end(), _p.begin(), _p.end());
    }

    segCentrPoints.points = multi_util::buildGraphPoints(fmm_segments.centroids.y, fmm_segments.centroids.x, fmm_segments.graph, mapMsg);

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

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    int ax, ay; // Dimensiones del agente
    Poss<int> agent, agent_; // Posiciones que ocupa el agente sobre el grid
    //ax = ay = ceil(2*0.35/mapMsg.info.resolution);
    ax = ceil(2*sX / mapMsg.info.resolution);
    ay = ceil(2*sY / mapMsg.info.resolution);
    ////cout<<ax<<", "<<ay<<endl;
    //cin.get();
    Poss<int> free_pos = positions[0]; // Todas las posiciones a ser alcanzadas

    // Variables que se usan en el método de exploración
    int vrange = (int)(laser_range/mapMsg.info.resolution); // Rango del laser (RECUERDA MIRAR EN STAGE) // ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
    vector<vector<int>> static_areas_map = fmm_segments.map;
    vector<vector<float>> static_map = grid;
    vector<Poss<int>> areas_poss = fmm_segments.poss;
    vector<vector<bool>> areas_graph = fmm_segments.graph;
    Poss<int> area_centroids = fmm_segments.centroids;
    int Nstatic_areas = area_centroids.x.size();
    exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);

    vector<Poss<int>> obstacle_clusters;
    dynamic_areas::Areas areas;
    vector<vector<bool>> trajectories_map;
    Poss<int> obstacles_trajectories_points;
    int number_of_visited_points = 0;

    // Inicializar lo que observa el agente (Sobre el grid)
    Poss<int> visible_poss, visible_obstacles, local_map_poss;
    Poss<int> visible_variation;
    vector<float> visible_values;
    vector<vector<float>> world_grid; // ESTE GRID CONTIENE LOS OBSTÁCULOS DINÁMICOS
    // POR AHORA ESTO, PERO LUEGO HABRÁ QUE AÑADIR LOS OBSTÁCULOS QUE NO ESTÁN EN EL MAPA ESTÁTICO
    world_grid = static_map;
    vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
    vector<float> laser_poss_values;
    
    vector<float> maxSegDist = fmm_segment::max_seg_dist(fmm_segments.centroids, fmm_segments.contour_poss);
    vector<vector<int>> tree;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    path2Goal.clear(); path2GoalPoints.clear(); path2GoalPoses.clear();

    bool goalRx = false, newGoal = true, goalResult;
    float dist = INF;
    float adist = INF;

    Poss<int> toExplore = expl_obj.get_all_poss2explore();

    bool explore = true;
    bool replan = false;

    //PathFollower pf("amcl_pose", "cmd_vel", maxLinearVel, maxAngularVel, 2*robotRadius, robotRadius, 0.2);
    //PathFollower pf("amcl_pose", "cmd_vel", laser_topic, maxLinearVel, maxAngularVel, 2*robotRadius, robotRadius, 1.8, 0.1, 1.0);

    int cont = 0;
    while(ros::ok() && explore){ // Hasta que explore todo el espacoio libre

        //cout<<"*************************************************************************************"<<endl;
        //cout<<"It: "<<cont<<endl;

        //cout<<"     -> El agente está en ("<<currPose.position.x<<", "<<currPose.position.y<<") | ("<<gridPosition.first<<", "<<gridPosition.second<<")"<<endl;

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //cout<<"método de exploración ..."<<endl;


        // LO que ve el explorador
        //laserPoss = fix_poss_in_map(laserPoss, grid);
        laserPoss = fix_poss_in_map(laserPoss, gsx, gsy);
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

        updateNavigableMap(navArea, navAreaOrig, laserPoss, visible_poss, ax, ay);
        
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

        //cout<<"Donde está el agente: ("<<gridPosition.first<<", "<<gridPosition.second<<"): "<<navArea[gridPosition.first][gridPosition.second]<<endl;

        // Calcular nuevo objetivo (partición + conjunto de puntos + goal) y el camino hacia él
        expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree);
        //cout<<"Objetivos y camino obtenidos"<<endl;

        //explore = expl_obj.get_target_area() >= 0;
        //toExplore = expl_obj.get_all_poss2explore();
        //explore = toExplore.x.size() > 0;
        explore = expl_obj.remaining_poss2explore();

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
                //cout<<"Replanificando ..."<<endl;
                vector<bool> dynamism_in_stareas; // Dinamismo en áreas dinámicas
                dynamism_in_stareas = dynamic_areas::in_area(areas.poss, static_areas_map, Nstatic_areas);
                //cout<<"Detectados nuevos obstáculos"<<endl;
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
                //cout<<"Posiciones no alcanzables descartadas"<<endl;
                expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree);
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
                    pathGradValues[i] /= maxv;
                    path2Follow[i].position.z = pathGradValues[i];
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

        followPath.poses = multi_util::poses2PoseStamped(path2Follow);
        followPath.header.seq = cont;
        followPath.header.frame_id = map_topic;
        followPath.header.stamp = ros::Time::now();
        pathFollowPub.publish(followPath); // Publico

        // Publicar el mapa de exploración
        //explorationMap.data = multi_util::grid2occupancyGridData(explored_map);
        //explorationMap.data = multi_util::map2occupancyGridData(navArea);
        explorationMap.data = multi_util::map2occupancyGridData(expl_obj.get_seen_map());
        explMapPub.publish(explorationMap); // Publico

        // Publicar los segmentos
        segmMapPub.publish(segmMap); // Publico

        // Publicar los puntos de los segmentos
        //segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 0.25;
        segFrontPub.publish(segFrontPoints);

        // Publicar los centroides de los segmentos con el grafo del entorno
        segCentrPoints.points = multi_util::buildTreePoints(fmm_segments.centroids.y, fmm_segments.centroids.x, tree, mapMsg);
        //segCentrPoints.color.r = 1.0; segCentrPoints.color.a = 0.25;
        segFrontPub.publish(segCentrPoints);

        // Publicar las posiciones objetivo del agente
        if(expl_obj.get_target_area() >= 0){
            Poss<int> _target = expl_obj.get_poss2explore()[expl_obj.get_target_area()];
            targetPoints.points = multi_util::tfGridPos2MapPos(mapMsg, _target.y, _target.x);
            //targetPoints.color.r = 1.0; targetPoints.color.a = 0.25;
            targetPosPub.publish(targetPoints);
        }
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        //cout<<"selecciono goal y mando al explorador ..."<<endl;
        // SELECCIONAR UN NUEVO GOAL
        /*
        if(newGoal){
            if(path2GoalPoses.size()){
                goalPose = path2GoalPoses[path2GoalPoses.size()-1]; // <---------- VOY AL FINAL DEL CAMINO Y YA ESTÁ
                goalGridPosition.first = path2Goal.x[path2Goal.x.size()-1]; goalGridPosition.second = path2Goal.y[path2Goal.y.size()-1]; 
            }
            newGoal = false;
            if(!goalRx){
                goalResult = sendGoal(goalPose); // Le paso el goal con la orientación
                // REALMENTE SE QUEDA AQUI ESPERANDO HASTA QUE SE LLEGA AL GOAL
                // ¿ES RENTABLE QUE SEA ASÍ? Por ahora, es la única manera para registrar "bien" lo explorado
                goalRx = true;
            }
        }
        */
        //newGoal = path2Goal.compare(_path2Goal);
        newGoal = goalGridPosition.first != expl_obj.getGoalx() || goalGridPosition.second != expl_obj.getGoaly();
        if(newGoal){ // Tengo un nuevo goal
            //cout<<"TENGO UN NUEVO GOAL"<<endl;
            /*
            actionGoal.goal.target_pose.pose = path2GoalPoses[0];
            while(path2GoalPoses.size()){
                pathPublisher.publish(actionGoal);
                path2GoalPoses.erase(path2GoalPoses.begin());
            }
            */
            if(path2GoalPoses.size()){
                goalPose = path2GoalPoses[path2GoalPoses.size()-1]; // <---------- VOY AL FINAL DEL CAMINO Y YA ESTÁ
                goalGridPosition.first = expl_obj.getGoalx(); goalGridPosition.second = goalGridPosition.second != expl_obj.getGoaly();
                //goalResult = sendGoal(goalPose); // Le paso el goal con la orientación
                /*
                // A partir de aqui seguir el camino con mi navegador
                while(ros::ok() && path2GoalPoses.size()){
                    computeVelocities(currPose, path2GoalPoses[0], maxLinearVel, maxAngularVel, cmd_vel.linear.x, cmd_vel.angular.z);
                    //computeVelocitiesDWA(currPose, path2GoalPoses[0], laserMsg, maxLinearVel, maxAngularVel, robotRadius, cmd_vel.linear.x, cmd_vel.angular.z);
                    //cout<<"enviando velocidades ("<<cmd_vel.linear.x<<", "<<cmd_vel.angular.z<<")"<<endl;
                    cmdVelPub.publish(cmd_vel);
                    ros::spinOnce();
                    loopRate.sleep();
                    //ros::Duration(0.5).sleep();
                    //if(checkDistance(currPose, path2GoalPoses[0], GOAL_DISTANCE, GOAL_ANGLE))
                        path2GoalPoses.erase(path2GoalPoses.begin());
                }
                */
                //pf.followPath(path2GoalPoses);
            }else{
                //cout<<"EL GOAL NO ES ALCANZABLE, HAY UN OBSTÁCULO EN EL GOAL"<<endl;
            }
            
        }

        //cout<<"     ->   El goal está en ("<<goalPose.position.x<<", "<<goalPose.position.y<<") | ("<<goalGridPosition.first<<", "<<goalGridPosition.second<<") → "<<navArea[goalGridPosition.first][goalGridPosition.second]<<endl;

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //cout<<"chequear llegada ..."<<endl;

        // CHEQUEAR SI SE HA LLEGADO AL GOAL
        
        ////cout<<"Expl → Goal: ("<<currPose.position.x<<", "<<currPose.position.y<<", "<<tf::getYaw(currPose.orientation)<<") → ("<<goalPose.position.x<<", "<<goalPose.position.y<<", "<<tf::getYaw(goalPose.orientation)<<"): "<<dist<<" - "<<adist<<endl;
        /*
        if(checkDistance(currPose, goalPose, GOAL_DISTANCE, GOAL_ANGLE)){ // HE LLEGADO AL OBJETIVO
            //cout<<"!!!!! HE LLEGADO AL GOAL, SE CALCULA UNO NUEVO !!!!!"<<endl;
            // Limpio las variables del camino
            path2Goal.clear(); path2GoalPoses.clear(); path2GoalPoints.clear();
            newGoal = true; // Voy a obtener el siguiente goal
            goalRx = false; // Todavia no se lo he enviado al robot
        }
        */

        //cout<<"Tamaños de caminos: "<<path2Goal.x.size()<<" - "<<path2GoalPoses.size()<<endl;
        //cout<<"¿QUÉ ESTOY HACIENDO? TENGO NUEVO GOAL?: "<<newGoal<<". LO HE RECIBIDO?: "<<goalRx<<". RESULTADO: "<<goalResult<<endl;
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        cont++;

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loopRate.sleep();

    }

    // Publicar camino vacío para que el path follower pare
    path2Follow.clear();
    followPath.poses = multi_util::poses2PoseStamped(path2Follow);
    followPath.header.seq = cont;
    followPath.header.frame_id = map_topic;
    followPath.header.stamp = ros::Time::now();
    pathFollowPub.publish(followPath); // Publico

    //cout<<"!!!!! WE DID IT BABY. BRING THE RED PANTIES !!!!!"<<endl;

    return 0;
}
