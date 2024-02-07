
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

// Para enviar goals al agente
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Para sincronizar los mensajes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



// TODAS LAS VARIABLES GLOBALES QUE VA A USAR EL EXPLORADOR

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // Para enviar los objetivos al robot

// Posiciones que usa el robot
pair<int, int> gridPosition, goalGridPosition, waypointGrid;
geometry_msgs::PoseWithCovarianceStamped realPose;
geometry_msgs::Pose currPose, goalPose, waypointPose;
geometry_msgs::Point explPoint, goalPoint, wayPoint;

// Posiciones del laser del explorador
vector<geometry_msgs::Point> laserPos, laserGridPos;
vector<geometry_msgs::Pose> laserPoses, laserGridPoses;
Poss<int> laserPoss;

// El grid del mapa (de ocupación → el que uso para planificar mis cosas: FMM, segmentar, path planning ... )
vector<vector<float>> grid;

// Mapa de lo explorado/no explorado
vector<vector<int>> explored_map;

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
    for(int i = 0; i<poses.size(); i++)
        cout<<"("<<poses[i].position.x<<", "<<poses[i].position.y<<", "<<poses[i].orientation.w<<")"<<endl;
}

Poss<int> agentPoss(int x, int y, int sx, int sy)
{
    Poss<int> res;
    for(int i=x-sx/2; i<=x+sx/2; i++)
        for(int j=y-sy/2; j<=y+sy/2; j++)
            res.push(i,j);
    return res; 
}

void updateExploredMap(pair<int, int> agent, Poss<int> laserPoss)
{
    int sx, sy;
    sx = explored_map.size();
    if(!sx) return;
    sy = explored_map[0].size();
    for(int i=0; i<laserPoss.x.size(); i++){
        //cout<<laserPoss.x[i]<<", "<<laserPoss.y[i]<<endl;
        Poss<int> p = bresenham::points(agent.first, agent.second, laserPoss.x[i], laserPoss.y[i]);
        for(int j=0; j<p.x.size(); j++){
            //cout<<"("<<p.x[j]<<", "<<p.y[j]<<")"<<endl;

            if(p.x[j]>=0 && p.x[j]<sx && p.y[j]>=0 && p.y[j]<sy){
                explored_map[p.x[j]][p.y[j]] = 1;
            }
        }
    }

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
    //goal.target_pose.header.frame_id = "base_link"; // Con respecto al robot
    goal.target_pose.header.frame_id = "map"; // Coordenada global
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
    //goal.target_pose.header.frame_id = "base_link"; // Con respecto al robot
    goal.target_pose.header.frame_id = "map"; // Coordenada global
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose = goalPose;

    
    ROS_INFO("Sending goal: (%f, %f, %f)", goalPose.position.x, goalPose.position.y, goalPose.orientation.w);
    ac.sendGoal(goal);

    return ac.waitForResult();
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

    tf::TransformListener listener;
    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped laserPoseStamped;
    laserPoseStamped.header.frame_id = "base_link";
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
            laserPoseStamped.header.stamp = ros::Time::now();
            laserPoseStamped.pose = pose;
            try{
                listener.transformPose("map", laserPoseStamped, transformedPose);
                laserGridPoses.push_back(transformedPose.pose);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("Error al transformar la pose del láser: %s", ex.what());
            }
            // *********************************************
        }
    }
    
}

void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
    currPose = robotPoseMsg->pose.pose;
}

void robotSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg, const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    // Obtener la pose del robot
    robotPoseCallback(pose_msg);
    
    // Obtener la lectura del laser
    laserCallback(laser_msg);
}


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

int main(int argc, char** argv) {
	ros::init(argc, argv, "explorer");
	ros::NodeHandle nh;

	// Get tf_prefix from the parameter server
    string tf_prefix;
	nh.getParam("tf_prefix", tf_prefix);


    // Para mover un obstáculo
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 0.5;

    //cmd_vel_pub.publish(vel_msg);
    //ros::spin();

    ros::Rate loopRate(0.5);

    currPose.position.x = currPose.position.y = -INF;

    // Me suscribo una vez
    
    //ros::Subscriber occGridSub = nh.subscribe("/map", 1, &occupancyMapCallback);
    //ros::Subscriber laserSub = nh.subscribe("/robot_0/base_scan", 1, &laserPosCallback);
    nav_msgs::OccupancyGrid map;
    vector<vector<int>> occGrid;
    Poss<int> obst;
    //ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(multi_util::_occupancyGridCallback, _1, boost::ref(occGrid)));
    //ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(multi_util::occupancyGridCallback, _1, boost::ref(map), boost::ref(occGrid)));
    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(multi_util::gridCallback, _1, boost::ref(map), boost::ref(occGrid), boost::ref(grid)));
    
    while(!grid.size()){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.1).sleep();  // Esperar un breve periodo de tiempo
        cout<<"ESPERANDO RECIBIR MAPA..." <<endl;
    }

    //ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, boost::bind(multi_util::laserPosCallback, _1, boost::ref(laserPos)));
    // → ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, boost::bind(multi_util::laserPosesCallback, _1, boost::ref(laserPoses)));
    
    //ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, &laserCallback);
    //ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &robotPoseCallback);

    // Definir los suscriptores para los mensajes de pose y láser scan
    //message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "/amcl_pose", 1);
    //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/base_scan", 1);

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "robot_0/odom", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "robot_0/base_scan", 1);

    // Definir la política de sincronización (en este caso, sincronización aproximada en base al tiempo)
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, laser_sub); // El número 10 es el tamaño del buffer

    // Establecer la función de callback para la sincronización de los mensajes
    sync.registerCallback(boost::bind(&robotSyncCallback, _1, _2));

    //ros::Subscriber cmdSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &checkUpdate);
    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &checkMov);

    bool save = true;

    while(currPose.position.x < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.1).sleep();  // Esperar un breve periodo de tiempo
        cout<<"ESPERANDO LOCALIZARME ..." <<endl;
    }

    explored_map.resize(grid.size(), vector<int>(grid[0].size(),0));

    ros::Time beginT, endT;

    Path camino; // Para todo
    Path path2Goal;
    vector<geometry_msgs::Point> path2GoalPoints;
    vector<geometry_msgs::Pose> path2GoalPoses;
    
    vector<vector<float>> obstGrad; // El gradiente desde los obstáculos
    vector<vector<float>> navArea; // Grid que se usa para la planificación de los movimientos del agente
    vector<vector<float>> grad; // Gradiente para todo
    vector<vector<float>> navGrad; // Gradiente que se usa para planificar/navegar (navGrad = grad → camino más cercano)
    fmm_segment::Segments fmm_segments;
    vector<Poss<int>> positions; // Posiciones libres y obstáculos

    if(grid.size()){
        // Posición del explorador
        //currPosition = multi_util::getRobotPosition();
        //currPose = multi_util::getRobotPose();
        gridPosition = multi_util::transformToGridCoordinates(map, currPose);
        cout<<"Explorer: ("<<currPose.position.x<<", "<<currPose.position.y<<") → ("<<gridPosition.first<<", "<<gridPosition.second<<")"<<endl;
        cout<<"OBSTACULO: "<<grid[gridPosition.first][gridPosition.second]<<endl;

        //return 0;

        // Calculo el gradiente desde la posición del agente para obtener el espacio libre en el que se va a mover
        beginT = ros::Time::now();
        FMM gr(gridPosition.first, gridPosition.second, grid);
        grad = gr.compute_gradient_();
        endT = ros::Time::now();
        cout<<"Gradiente: "<<endT - beginT<<endl;

        // Cargarme las posiciones "no alcanzables" del grid
        positions = fix_free_space(grid, grad);

        // Gradiente de los obstáculos
        beginT = ros::Time::now();
        FMM ogr(positions[1].x, positions[1].y, grid);
        obstGrad = ogr.compute_gradient_();
        endT = ros::Time::now();
        cout<<"Gradiente obstáculos: "<<endT - beginT<<endl;

        navArea = multi_util::navigableGradient(obstGrad, 0.5/map.info.resolution);
        // Cargarme las posiciones que están cerca de los obstáculos
        positions = fix_free_space(grid, navArea);

        // Goal "random" → al centro de la habitación más grande del escenario
        find_matr_max(navArea, goalGridPosition.first, goalGridPosition.second);
        goalPose = multi_util::transformFromGridCoordinates(map, goalGridPosition);    
        cout<<"Goal: ("<<goalGridPosition.first<<", "<<goalGridPosition.second<<") → ("<<goalPose.position.x<<", "<<goalPose.position.y<<")"<<endl;

        // Gradiente desde el robot
        beginT = ros::Time::now();
        FMM cgr(gridPosition.first, gridPosition.second, navArea); // Voy a alejar al agente de los obstáculos
        navGrad = cgr.compute_gradient_();
        endT = ros::Time::now();
        cout<<"Gradiente desviado: "<<endT - beginT<<endl;

        // Calcular el camino hasta el goal
        path2Goal.gradient_descent_(navGrad, goalGridPosition.first, goalGridPosition.second);
        path2Goal.show();

        // Pasar camino en el grid a puntos del mapa
        path2GoalPoints = multi_util::tfGridPos2MapPos(map, path2Goal.y, path2Goal.x);
        //showPoints(path2GoalPoints);
        path2GoalPoses = multi_util::convertPointsToPoses(path2GoalPoints);
        //showPoses(path2GoalPoses);

        // Segmentar el entorno
        fmm_segments = fmm_segment::compute_segments(grid, navArea);
        for(int i=0; i<fmm_segments.frontier_poss.size(); i++) fmm_segments.contour_poss[i] = geometry::sort_points(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i]);
        endT = ros::Time::now();
        cout<<"Entorno partido: "<<endT - beginT<<endl;

        if(save){
            // ESTO ES PARA REPRESENTAR CON Python
            cout<<"ORIGEN DEL MAPA: ("<<map.info.origin.position.x<<", "<<map.info.origin.position.y<<") - "<<map.info.resolution<<endl;
            //laserPoses = multi_util::laserPosesTf(currPose, laserPoses); // tf a las coordenadas del mapa
            //laserPos = laserTf(laserPos, currPosition);
            //cout<<"Laser tf"<<endl; showPoints(laserPos);
            //laserGridPos = multi_util::transform2GridCoordinates(map, multi_util::laserPosesTf(currPose, laserPoses));
            laserGridPos = multi_util::transform2GridCoordinates(map, laserGridPoses);
            laserPoss = points2Poss(laserGridPos);
            //laserPoss.show();
            if(!moving) updateExploredMap(gridPosition, laserPoss);  

            //cout<<"laser"<<endl;

            //save = false;
            Poss<int> expl; expl.push(gridPosition.first, gridPosition.second); expl.push(goalGridPosition.first, goalGridPosition.second);
            expl.save("explorer.txt");
            obst.save("obstaculos.txt");
            save_matr("occGrid.txt",occGrid);
            save_matr("miGrid.txt",grid);
            save_matr("grad.txt",grad);
            save_matr("obstGrad.txt",obstGrad);
            save_matr("navArea.txt",navArea);
            save_matr("navGrad.txt",navGrad);
            save_matr("explored_map.txt",explored_map);
            save_matr("segments_map.txt",fmm_segments.map);
            path2Goal.save("path.txt");
            laserPoss.save("laser.txt");
        }
        
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
    laserGridCell.header.frame_id = "base_link";
    laserGridCell.cell_width = 4*map.info.resolution;
    laserGridCell.cell_height = 4*map.info.resolution;
    laserGridCell.cells = poses2Points(laserPoses);

    // Para publicar el camino planificado con FMM
    // (LO SUYO SERÍA DIRECTAMENTE PASAR ESTE CAMINO AL STACK DE NAVEGACIÓN DE ROS)
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("pathToGoal", 10);
    nav_msgs::Path rosPath;
    rosPath.header.seq = 0;
    rosPath.header.stamp = ros::Time::now();
    rosPath.header.frame_id = "map";
    //rosPath.poses = multi_util::transformFromGridCoordinates(map, path2Goal.y, path2Goal.x);
    rosPath.poses = multi_util::poses2PoseStamped(path2GoalPoses);

    // Para publicar el mapa de exploración
    ros::Publisher explMapPub = nh.advertise<nav_msgs::OccupancyGrid>("explorationMap", 10);
    nav_msgs::OccupancyGrid explorationMap;
    explorationMap.header.seq = 0;
    explorationMap.header.stamp = ros::Time::now();
    explorationMap.header.frame_id = "map";
    explorationMap.info = map.info; // Debería ser lo mismo
    explorationMap.data = multi_util::grid2occupancyGridData(explored_map);

    // Para publicar el mapa de los segmentos
    ros::Publisher segmMapPub = nh.advertise<nav_msgs::OccupancyGrid>("segmentsMap", 10);
    nav_msgs::OccupancyGrid segmMap;
    segmMap.header.seq = 0;
    segmMap.header.stamp = ros::Time::now();
    segmMap.header.frame_id = "map";
    segmMap.info = map.info; // Debería ser lo mismo
    segmMap.data = multi_util::map2occupancyGridData(fmm_segments.map);

    // Para publicar las fronteras de los segmentos
    ros::Publisher segFrontPub = nh.advertise<visualization_msgs::Marker>("segFront", 10);
    visualization_msgs::Marker segFrontPoints, segCentrPoints;
    segFrontPoints.id = 0;
    segCentrPoints.id = 1;
    segFrontPoints.header.seq = segCentrPoints.header.seq = 0;
    segFrontPoints.header.stamp = segCentrPoints.header.stamp = ros::Time::now();
    segFrontPoints.header.frame_id = segCentrPoints.header.frame_id = "map";
    segFrontPoints.ns = segCentrPoints.ns = "segPoints";
    segFrontPoints.action = segCentrPoints.action = visualization_msgs::Marker::ADD;
    segFrontPoints.pose.orientation.w = segCentrPoints.pose.orientation.w = 1;
    segFrontPoints.type = visualization_msgs::Marker::POINTS;
    segCentrPoints.type = visualization_msgs::Marker::LINE_LIST;
    segFrontPoints.scale.x = segFrontPoints.scale.y = segCentrPoints.scale.x = segCentrPoints.scale.y = map.info.resolution;
    segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 1.0;
    segCentrPoints.color.r = 1.0; segCentrPoints.color.a = 1.0;
    vector<geometry_msgs::Point> _p;
    for(int i=0; i<fmm_segments.contour_poss.size(); i++){
        _p = multi_util::tfGridPos2MapPos(map, fmm_segments.contour_poss[i].y, fmm_segments.contour_poss[i].x);
        segFrontPoints.points.insert(segFrontPoints.points.end(), _p.begin(), _p.end());
    }

    segCentrPoints.points = multi_util::buildGraphPoints(fmm_segments.centroids.y, fmm_segments.centroids.x, fmm_segments.graph, map);

    // Para publicar los objetivos del agente
    ros::Publisher targetPosPub = nh.advertise<visualization_msgs::Marker>("targetPos", 10);
    visualization_msgs::Marker targetPoints;
    targetPoints.id = 0;
    targetPoints.header.seq = 0;
    targetPoints.header.stamp = ros::Time::now();
    targetPoints.header.frame_id = "map";
    targetPoints.ns = "targetPoints";
    targetPoints.action = visualization_msgs::Marker::ADD;
    targetPoints.pose.orientation.w = 1;
    targetPoints.type = visualization_msgs::Marker::POINTS;
    targetPoints.scale.x = targetPoints.scale.y = map.info.resolution;
    targetPoints.color.r = 1.0; targetPoints.color.a = 0.5;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Variables para seguir los caminos
    float length = 0; // Longitud "estimada" del camino (nº celdas x resolución del mapa)
    float delta = 0; // Distancia (metros) a la que se fija el siguiente waypoint
    int pathStep = 0; // Cantidad de celdas que se van a recorrer hasta el siguiente waypoint
    int nextWaypointInd = 0; // Índice (de la posición del camino) del siguiente waypoint a alcanzar
    delta = 300; // Fijo como recorrido máximo 1m para no desviarme mucho del camino

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    int ax, ay; // Dimensiones del agente
    Poss<int> agent; // Posiciones que ocupa el agente sobre el grid
    ax = ay = (int)(0.35/map.info.resolution);
    cout<<ax<<", "<<ay<<endl;
    Poss<int> free_pos = positions[0]; // Todas las posiciones a ser alcanzadas

    // Variables que se usan en el método de exploración
    int vrange = (int)(30.0/map.info.resolution); // Rango del laser (RECUERDA MIRAR EN STAGE) // ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
    vector<vector<int>> static_areas_map = fmm_segments.map;
    vector<vector<float>> static_map = grid;
    vector<Poss<int>> areas_poss = fmm_segments.poss;
    vector<vector<bool>> areas_graph = fmm_segments.graph;
    Poss<int> area_centroids = fmm_segments.centroids;
    exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);

    // Inicializar lo que observa el agente (Sobre el grid)
    Poss<int> visible_poss, visible_obstacles, local_map_poss;
    vector<vector<float>> world_grid; // ESTE GRID CONTIENE LOS OBSTÁCULOS DINÁMICOS
    // POR AHORA ESTO, PERO LUEGO HABRÁ QUE AÑADIR LOS OBSTÁCULOS QUE NO ESTÁN EN EL MAPA ESTÁTICO
    world_grid = static_map;
    vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
    
    vector<float> maxSegDist = fmm_segment::max_seg_dist(fmm_segments.centroids, fmm_segments.contour_poss);
    vector<vector<int>> tree;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    path2Goal.clear(); path2GoalPoints.clear(); path2GoalPoses.clear();

    bool goalRx = false, newGoal = false, goalResult;
    float dist = INF;
    float adist = INF;

    Poss<int> toExplore = expl_obj.get_all_poss2explore();

    bool explore = true;

    int cont = 0;
    while(ros::ok() && explore){ // Hasta que explore todo el espacoio libre

        cout<<"*************************************************************************************"<<endl;
        cout<<"It: "<<cont<<endl;
        cout<<expl_obj.get_all_poss2explore().x.size()<<" posiciones por ver"<<endl;

        // Actualizar la posición del explorador sobre el grid
        gridPosition = multi_util::transformToGridCoordinates(map, currPose);

        cout<<"     -> El agente está en ("<<currPose.position.x<<", "<<currPose.position.y<<") | ("<<gridPosition.first<<", "<<gridPosition.second<<")"<<endl;

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        cout<<"actualizo mapa ..."<<endl;

        // ACTUALIZAR EL MAPA DE LO EXPLORADO
        // Obtener las posiciones del laser sobre el grid
        laserGridPos = multi_util::transform2GridCoordinates(map, laserGridPoses);
        laserPoss = points2Poss(laserGridPos);
        // Registro el mapa de lo explorado unicamente cuando estoy quieto, para "no cometer errores"
        // SOLUCIÓN MUY LAMENTABLE, PERO POR AHORA PUEDE TIRAR
        if(!moving) updateExploredMap(gridPosition, laserPoss);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        cout<<"método de exploración ..."<<endl;

        // LO que ve el explorador
        visible_poss = allLaserPoss(gridPosition.first, gridPosition.second, laserPoss);
        //cout<<"Posiciones del láser sibre el grid obtenidas"<<endl;
        // Obtener las posiciones que ocupa el agente sobre el grid
        agent = agentPoss(gridPosition.first, gridPosition.second, ax, ay);
        //cout<<"EL AGENTE OCUPA LAS POSICIONES: "<<endl; agent.show();
        // Añadir las posiciones del agente a las visibles
        visible_poss.append(agent);
        //cout<<"Posiciones del agente añadidas"<<endl;

        // Esto es por si estoy "mal" localizado y las posiciones del láser se salen del grid
        visible_poss = fix_poss_in_map(visible_poss, grid);

        //laserPoss = points2Poss(laserGridPos);
        map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo
        //sh_vect_h(map_values,"Map_values");
        //cout<<"Obtenido lo que ve el explorador"<<endl;

        // Actualizar las variables de exploración (mapas basicamente)
        expl_obj.update_all(visible_poss, map_values);
        //cout<<"Actualizado en el método de exploración"<<endl;

        // SELECCIÓN DE LOS NUEVOS OBJETIVOS
        // Reiniciar los objetivos
        expl_obj.update_targets();
        //cout<<"Objetivos reiniciados"<<endl;
        // Calcular nuevo objetivo (partición + conjunto de puntos + goal) y el camino hacia él
        expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree);
        //cout<<"Objetivos y camino obtenidos"<<endl;

        toExplore = expl_obj.get_all_poss2explore();

        explore = expl_obj.get_target_area() >= 0;

        cout<<"HE seleccionado segmento "<<expl_obj.get_target_area()<<", conjunto "<<expl_obj.get_target_set()<<endl;

        if(path2Goal.x.size()){
            // Pasar camino en el grid a puntos del mapa
            path2GoalPoints = multi_util::tfGridPos2MapPos(map, path2Goal.y, path2Goal.x);
            //showPoints(path2GoalPoints);
            path2GoalPoses = multi_util::convertPointsToPoses(path2GoalPoints);
            //showPoses(path2GoalPoses);
        }
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        cout<<"publico topics ..."<<endl;

        // PUBLICAR LOS TOPICS DE LAS COSAS QUE QUIERO REPRESENTAR
        // Publicar las celdas para representar sobre el mapa en RViz
        laserGridCell.cells = poses2Points(laserPoses); // Actualizar lo que voy a publicar
        laserGridCell.header.stamp = ros::Time::now();
        laserPub.publish(laserGridCell); // Publico

        // Publicar el camino que sigue el robot
        rosPath.poses = multi_util::poses2PoseStamped(path2GoalPoses);
        rosPath.header.stamp = ros::Time::now();
        pathPub.publish(rosPath); // Publico

        // Publicar el mapa de exploración
        //explorationMap.data = multi_util::grid2occupancyGridData(explored_map);
        explorationMap.data = multi_util::map2occupancyGridData(expl_obj.get_seen_map());
        explMapPub.publish(explorationMap); // Publico

        // Publicar los segmentos
        segmMapPub.publish(segmMap); // Publico

        // Publicar los puntos de los segmentos
        //segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 0.25;
        segFrontPub.publish(segFrontPoints);

        // Publicar los centroides de los segmentos con el grafo del entorno
        segCentrPoints.points = multi_util::buildTreePoints(fmm_segments.centroids.y, fmm_segments.centroids.x, tree, map);
        //segCentrPoints.color.r = 1.0; segCentrPoints.color.a = 0.25;
        segFrontPub.publish(segCentrPoints);

        // Publicar las posiciones objetivo del agente
        if(expl_obj.get_target_area() >= 0){
            Poss<int> _target = expl_obj.get_poss2explore()[expl_obj.get_target_area()];
            targetPoints.points = multi_util::tfGridPos2MapPos(map, _target.y, _target.x);
            //targetPoints.color.r = 1.0; targetPoints.color.a = 0.25;
            targetPosPub.publish(targetPoints);
        }
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        cout<<"selecciono goal y mando al explorador ..."<<endl;
        // SELECCIONAR UN NUEVO GOAL
        if(newGoal){
            if(path2GoalPoses.size()){
                goalPose = path2GoalPoses[path2GoalPoses.size()-1]; // <---------- VOY AL FINAL DEL CAMINO Y YA ESTÁ
                goalGridPosition.first = path2Goal.x[path2Goal.x.size()-1]; goalGridPosition.second = path2Goal.y[path2Goal.y.size()-1]; 
            }
            newGoal = false;
            if(!goalRx){
                goalResult = sendGoal(goalPose); // Le paso el goal con la orientación
                // REALMENTE SE QUEDA AQUI ESPERANDO HGASTA QUE SE LLEGA AL GOAL
                // ¿ES RENTABLE QUE SEA ASÍ? Por ahora, es la única manera para registrar "bien" lo explorado
                goalRx = true;
            }
        }

        cout<<"     ->   El goal está en ("<<goalPose.position.x<<", "<<goalPose.position.y<<") | ("<<goalGridPosition.first<<", "<<goalGridPosition.second<<") → "<<navArea[goalGridPosition.first][goalGridPosition.second]<<endl;

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        cout<<"chequear llegada ..."<<endl;

        // CHEQUEAR SI SE HA LLEGADO AL GOAL
        
        //cout<<"Expl → Goal: ("<<currPose.position.x<<", "<<currPose.position.y<<", "<<tf::getYaw(currPose.orientation)<<") → ("<<goalPose.position.x<<", "<<goalPose.position.y<<", "<<tf::getYaw(goalPose.orientation)<<"): "<<dist<<" - "<<adist<<endl;
        if(checkDistance(currPose, goalPose, GOAL_DISTANCE, GOAL_ANGLE)){ // HE LLEGADO AL OBJETIVO
            cout<<"!!!!! WE DID IT BABY. BRING THE RED PANTIES !!!!!"<<endl;
            // Limpio las variables del camino
            path2Goal.clear(); path2GoalPoses.clear(); path2GoalPoints.clear();
            newGoal = true; // Voy a obtener el siguiente goal
            goalRx = false; // Todavia no se lo he enviado al robot
        }

        cout<<"Tamaños de caminos: "<<path2Goal.x.size()<<" - "<<path2GoalPoses.size()<<" : "<<pathStep<<endl;
        cout<<"¿QUÉ ESTOY HACIENDO? TENGO NUEVO GOAL?: "<<newGoal<<". LO HE RECIBIDO?: "<<goalRx<<". RESULTADO: "<<goalResult<<endl;
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        cont++;

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loopRate.sleep();

    }

	return 0;
}
