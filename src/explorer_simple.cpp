
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

    totalPathPose.pose = currPose;
    totalPath.poses.push_back(totalPathPose);
}

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    //ros::Duration(1.0).sleep();
    //cout<<"SUSCRITO CON odom"<<endl;
    currPose = robotPoseMsg->pose.pose;
    gridPosition = multi_util::transformToGridCoordinates(mapMsg, currPose);

    robotOdomPoseMsg = *robotPoseMsg;

    totalPathPose.pose = currPose;
    totalPath.poses.push_back(totalPathPose);
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

Poss<int> dynObstPos(Poss<int> laser_poss, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> grid, int x, int y, float vrange, vector<geometry_msgs::Point> &obst_pos)
{
    Poss<int> res;
    float d;
    for(int i=0; i<laser_poss.x.size(); i++){
        if(grid[laser_poss.x[i]][laser_poss.y[i]]){
            d = hypot(x - laser_poss.x[i], y - laser_poss.y[i]);
            if(abs(d-vrange)>3){
                res.push(laser_poss.x[i], laser_poss.y[i]);
                obst_pos.push_back(laser_poses[i].position);
            }
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


float distance2closest_obstacle(int x, int y, Poss<int> obstacles)
{
    float res = INFINITY, d;
    for(int i=0; i<obstacles.x.size(); i++){
        d = hypot(x - obstacles.x[i], y - obstacles.y[i]);
        if(res > d) res = d;
    }
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

    //nav_msgs::OccupancyGrid map;
    vector<vector<int>> occGrid;
    Poss<int> obst;
    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::gridCallback, _1, boost::ref(mapMsg), boost::ref(occGrid), boost::ref(grid)));

    // Subscriber laser
    LaserScanToPointCloud lstopc(nh, map_topic);
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

    while(!grid.size() || currPose.position.x < -10000 || gridPosition.first < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.1).sleep();  // Esperar un breve periodo de tiempo
        //cout<<"ESPERANDO MAPA Y LOCALIZARME ..." <<endl;
    }

    vector<vector<float>> original_grid = grid;

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
    vector<vector<float>> navAreaOrigNorm; // El original normalizado
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
        positions = fix_free_space_(grid, navArea);

        // Goal "random" → al centro de la habitación más grande del escenario
        find_matr_max(navArea, goalGridPosition.first, goalGridPosition.second);
        goalPose = multi_util::transformFromGridCoordinates(mapMsg, goalGridPosition);

        // Gradiente desde el robot
        beginT = ros::Time::now();
        FMM cgr(gridPosition.first, gridPosition.second, navArea); // Voy a alejar al agente de los obstáculos
        navGrad = cgr.compute_gradient_();
        endT = ros::Time::now();

        fix_free_space_(grid, navGrad);

        /*
        // Calcular el camino hasta el goal
        path2Goal.gradient_descent_(navGrad, goalGridPosition.first, goalGridPosition.second);
        path2Goal.show();

        // Pasar camino en el grid a puntos del mapa
        path2GoalPoints = multi_util::tfGridPos2MapPos(mapMsg, path2Goal.y, path2Goal.x);
        path2GoalPoses = multi_util::convertPointsToPoses(path2GoalPoints);
        */

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

    }

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
    //exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
    int Nposs2explore = 0;
    for(int i=0; i<areas_poss.size(); i++)
        Nposs2explore += areas_poss[i].x.size();
    
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
    vector<Poss<int>> clusters;
    Poss<int> obstacles_trajectories_points;
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

    vector<int> number_of_obstacles;
    int max_obst = 0;


    vector<vector<float>> seen_time(gsx, vector<float> (gsy, -1)); // Mapa con los instantes de tiempo en los que se ha visto cada posición del grid
    vector<float> maxSegDist = fmm_segment::max_seg_dist(fmm_segments.centroids, fmm_segments.contour_poss);

    vector<bool> dynamism(fmm_segments.centroids.x.size(), false);

    cout<<"variables inutiles"<<endl;

    //save_matr("static_map.txt", static_map);

    exploration::Learn expl_obj(static_map, static_map, vrange);
    map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo

    cout<<map_values.size()<<endl;
    expl_obj.update_maps(gridPosition.first, gridPosition.second, visible_poss, map_values); // Actualizar las utilidades en base a lo que se ve
    cout<<"mapas explorador actualizados"<<endl;
    //expl_obj.initialize_estimated_map();

    cout<<"variables explorer"<<endl;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    path2Goal.clear(); path2GoalPoints.clear(); path2GoalPoses.clear();

    bool goalRx = false, newGoal = true, goalResult;
    float dist = INF;
    float adist = INF;

    //Poss<int> toExplore = expl_obj.get_all_poss2explore();

    pair<float, float> poseDist;

    float goalTimeout = 5.0, _goalTimeout;
    ros::Time timeoutBegin, timeoutEnd;

    ros::Time currTimeBegin, currTimeEnd;

    bool explore = true;
    bool replan = false;

    float rot = 0;

    cout<<"COMENZANDO ................................................."<<endl;

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

    int wait_it = 0;
    
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

        // Si estoy mal localizado, descarto la lectura del láser
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
        //updateNavigableMap(navArea, navAreaOrig, laserPoss, visible_poss, ax, ay);
        dynamic_areas::_updateNavigableMap(navArea, navAreaOrig, laserPoss, visible_poss, ax, ay, gridPosition.first, gridPosition.second, vrange);

        // Extraer unicamente los obstáculos dinámicos
        //visible_obstacles_poses.clear();
        visible_obstacles_pos.clear();
        //visible_obstacles = dynObstPos(laserPoss, laserPoses, navAreaOrig, visible_obstacles_pos);
        //visible_obstacles = dynObstPos(laserPoss, laserPoses, navAreaOrig, visible_poss, navArea, visible_obstacles_pos);
        visible_obstacles = dynObstPos(laserPoss, laserPoses, navAreaOrig, gridPosition.first, gridPosition.second, vrange, visible_obstacles_pos);

        //save_matr("seen_time.txt", seen_time);

        //sh_vect_h(estimated_distance, "d");
        //sh_vect_h(estimated_velocity, "v");
        //sh_vect_h(forgetting_time, "forgetting_time");
        //sh_vect_h(last_time, "last_time");



        // -----------------------------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------------------------

        // Insertar la información de las áreas dinámicas en el mapa para planificar/navegar
        for(int i=0; i<visible_obstacles.x.size(); i++){
            navArea[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;
        }

        //save_matr("navArea.txt", navArea);

        // Actualizar las variables de exploración (mapas basicamente)
        expl_obj.update_maps(gridPosition.first, gridPosition.second, visible_poss, map_values);

        // Obtener camino al goal
        path2Goal = expl_obj.path2next_goal(gridPosition.first, gridPosition.second, navArea);

        //cout<<"exporar "<<explore<<endl;

        //cout<<"HE seleccionado segmento "<<expl_obj.get_target_area()<<", conjunto "<<expl_obj.get_target_set()<<endl;

        replan = false;
        if(path2Goal.x.size()){

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
        explMapPub.publish(explorationMap); // Publico

        // Publicar los centroides de los segmentos con el grafo del entorno
        //segCentrPoints.points = multi_util::buildTreePoints(fmm_segments.centroids.y, fmm_segments.centroids.x, tree, mapMsg);
        //segCentrPoints.color.r = 1.0; segCentrPoints.color.a = 0.25;
        //segFrontPub.publish(segCentrPoints);

        //explore = expl_obj.Nremaining_poss2explore() > 10;
        //explore = expl_obj.Nremaining_poss2explore() > (Nposs2explore * 0.0001);

        //explore = path2Goal.tam > 0;
        if(path2Goal.tam == 0){
            wait_it++;
        }else{
            wait_it = 0;
        }

        if(wait_it == 20){
            explore = false;
            intrusions++;
            collisions++;
        }

        // Publicar las posiciones objetivo del agente
        if(path2Goal.tam){
            Poss<int> _target;
            _target.push(path2Goal.x[path2Goal.x.size()-1], path2Goal.y[path2Goal.y.size()-1]);
            targetPoints.points = multi_util::tfGridPos2MapPos(mapMsg, _target.y, _target.x);
            //targetPoints.color.r = 1.0; targetPoints.color.a = 0.25;
            targetPosPub.publish(targetPoints);
        }

        // Publicar los obstáculos dinámicos vistos por el agente
        dynObstMarker.points = visible_obstacles_pos;
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

        cout<<"CAMINO: "<<path2Goal.tam<<" - "<<path2Follow.size()<<endl;

        // Almacenar variables de los resultados
        Nposs2explore_tot.push_back(expl_obj.Nremaining_poss2explore());
        dist2dynamic_obstacles.push_back(distance2closest_obstacle(gridPosition.first, gridPosition.second, visible_obstacles) * mapMsg.info.resolution);
        dist2static_obstacles.push_back(obstGrad[gridPosition.first][gridPosition.second] * mapMsg.info.resolution);
        travelled_distance.push_back(multi_util::computePathDistance(totalPath));

        computationTime.push_back(computationTimeEnd.toSec() - computationTimeBegin.toSec());

        //cout<<totalPath.poses.size()<<" puntos recorridos"<<endl;
        //cout<<"Distancia recorrida: "<<multi_util::computePathDistance(totalPath)<<endl;
        cout<<"Distancia recorrida: "<<travelled_distance[travelled_distance.size() - 1]<<endl;
        cout<<"Puntos por explorar: "<<Nposs2explore_tot[Nposs2explore_tot.size() - 1]<<" | "<<Nposs2explore<<endl;
        cout<<"Distancia al obstáculo: "<<dist2dynamic_obstacles[dist2dynamic_obstacles.size() - 1]<<" | "<<dist2static_obstacles[dist2static_obstacles.size() - 1]<<endl;
        cout<<"TIEMPO RESTANTE DE MISSIÓN: "<<max_mission_time<<endl;
        cout<<"Tiempo de cálculo: "<<computationTime[computationTime.size()-1]<<endl;

    }

    endTime = ros::Time::now();
    missionTime = endTime.toSec() - beginTime.toSec();

    float explored = exploration::explored_area(expl_obj.get_seen_map(), navAreaOrig);

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

    return 0;
}
