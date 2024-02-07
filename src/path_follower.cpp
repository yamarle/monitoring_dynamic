#include <multi_dynamic/path_follower.hpp>

PathFollower::PathFollower()
{
	//n_ = nh;
} // Constructor por defecto

PathFollower::~PathFollower(){}

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, double sX, double sY, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double posDiff, double angleDiff, string planner_type, double angularResolution, double maxObstacleDistance, string footprintTopic, string planTopic)
{

    goalRefTime = sleepTime;

    if(planner_type.compare("simple") == 0){
        ROS_INFO("path follower simple seleccionado");
        initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
        //robotRadius = std::max(sX,sY);
        robotRadius = sqrt(pow(sX,2)+pow(sY,2));
        // Distancia (máxima) a la que se considera que se va a encontrar el goal
        goalDist = max(maxLinearVel * goalRefTime + robotRadius, 2*robotRadius);
        initializeVariables(maxLinearVel, maxAngularVel, linearAcc, angularAcc, posDiff, angleDiff);
        initializePlotting(footprintTopic, sX, sY, planTopic);
    }else if(planner_type.compare("dwa") == 0){
        ROS_INFO("path follower con DWA seleccionado");
        dwaPlanner = true;
        initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic, laserTopic);
        //robotRadius = std::max(sX,sY);
        robotRadius = sqrt(pow(sX,2)+pow(sY,2));
        initializeVariables(maxLinearVel, maxAngularVel, linearAcc, angularAcc, posDiff, angleDiff, angularResolution, maxObstacleDistance);
        initializePlotting(footprintTopic, sX, sY, planTopic);
    }else{
        ROS_INFO("TIPO DE PLANIFICADOR NO SELECCIONADO !!!");
    }
} // Constructor para seguimiento simple

geometry_msgs::PolygonStamped PathFollower::computeFootprint(geometry_msgs::Pose pose, double sX, double sY)
{
    geometry_msgs::PolygonStamped res;
    res.header.stamp = ros::Time::now();
    //res.header.frame_id = "base_link"; // <---------------------------- PASAR FRAME CORRECTO DEL ROBOT
    res.header.frame_id = baseFrame;
    //res.header.frame_id = path.header.frame_id;
    geometry_msgs::Point32 p;
    p.x = -sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    p.x = -sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    return res;
}

nav_msgs::Path PathFollower::setPlan(geometry_msgs::Pose s, geometry_msgs::Pose g)
{
    nav_msgs::Path res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "map";

    geometry_msgs::PoseStamped p;
    p.pose = s; res.poses.push_back(p);
    g.position.z = 0.0; // ESTO ES PARA CUANDO EL CAMINO NO ES 3D
    p.pose = g; res.poses.push_back(p);

    //cout<<"("<<s.position.x<<", "<<s.position.y<<") --> ("<<g.position.x<<", "<<g.position.y<<")"<<endl;

    return res;
}

void PathFollower::initializePlotting(string footprintTopic, double sX, double sY, string planTopic)
{
    footprintPub = n_.advertise<geometry_msgs::PolygonStamped>(footprintTopic, 10);
    this->sX = sX; this->sY = sY;
    planPub = n_.advertise<nav_msgs::Path>(planTopic, 10);
} // Inicializar 

pair<double, double> PathFollower::poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    pair<double, double> res;
    res.first = sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
    double o1 = tf::getYaw(p1.orientation);
    double o2 = tf::getYaw(p2.orientation);
    res.second = abs(atan2(sin(o1-o2), cos(o1-o2)));
    return res;
}

double PathFollower::angle(geometry_msgs::Pose sp, geometry_msgs::Pose gp)
{
    double angleDiff = atan2(gp.position.y - sp.position.y, gp.position.x - sp.position.x) - tf::getYaw(sp.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    return angleDiff;
}

bool PathFollower::checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    pair<double, double> dist = poseDistance(p1,p2);
    if(dist.first <= goalDistDiff && dist.second <= goalAngleDiff) return true;
    return false;
}

int PathFollower::findClosest(const geometry_msgs::Pose pose, vector<geometry_msgs::Pose> poses)
{
    int res = -1;
    double d = INFINITY, _d = 0;
    for(int i=0; i<poses.size(); i++){
        _d = hypot(pose.position.x - poses[i].position.x, pose.position.y - poses[i].position.y);
        if(_d < d){
            d = _d;
            res = i;
        }
    }

    return res;
}

geometry_msgs::Pose PathFollower::nextGoal(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double size)
{
    // Estoy suponiendo que el agente se encuentra sobre el camino (en la primera posición)
    geometry_msgs::Pose res;

    while(poses.size()){
        //for(int i = 0; i<poses.size(); i++)
        //cout<<poses[0].position.z<<" - "<<robotRadius<<endl;
        //cout<<"CAMINO: ("<<poses[0].position.x<<", "<<poses[0].position.y<<") - ("<<poses[poses.size()-1].position.x<<", "<<poses[poses.size()-1].position.y<<")"<<endl;
        if(sqrt(pow(pose.position.x - poses[0].position.x, 2) + pow(pose.position.y - poses[0].position.y, 2)) <= size){
            //cout<<"("<<pose.position.x<<", "<<pose.position.y<<")"<<" Elmino ("<<poses[0].position.x<<", "<<poses[0].position.y<<")"<<endl;
            //cout<<sqrt(pow(pose.position.x - poses[0].position.x, 2) + pow(pose.position.y - poses[0].position.y, 2))<<" <= "<<size<<endl;
            //cout<<"a: "<<poses.size()<<endl;
            poses.erase(poses.begin());
            //cout<<"d: "<<poses.size()<<endl;
        }else{
            //cout<<"GOAL: ("<<poses[0].position.x<<", "<<poses[0].position.y<<")"<<endl;
            return poses[0];
        }
    }
    return res;
}

geometry_msgs::Pose PathFollower::nextGoal(geometry_msgs::Pose pose, vector<geometry_msgs::PoseStamped>& poses, double size)
{
    // Estoy suponiendo que el agente se encuentra sobre el camino (en la primera posición)
    geometry_msgs::Pose res;
    
    while(poses.size()){
        if(sqrt(pow(pose.position.x - poses[0].pose.position.x, 2) + pow(pose.position.y - poses[0].pose.position.y, 2)) <= size){
            //cout<<"("<<pose.position.x<<", "<<pose.position.y<<")"<<" Elmino ("<<poses[0].position.x<<", "<<poses[0].position.y<<")"<<endl;
            //cout<<sqrt(pow(pose.position.x - poses[0].position.x, 2) + pow(pose.position.y - poses[0].position.y, 2))<<" <= "<<size<<endl;
            //cout<<"a: "<<poses.size()<<endl;
            poses.erase(poses.begin());
            //cout<<"d: "<<poses.size()<<endl;
        }else{
            //cout<<"GOAL: ("<<poses[0].position.x<<", "<<poses[0].position.y<<")"<<endl;
            return poses[0].pose;
        }
    }
    return res;
}

geometry_msgs::Pose PathFollower::selGoal_base(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght)
{
    // Estoy suponiendo que el agente se encuentra sobre el camino (en la primera posición)
    geometry_msgs::Pose res;

    //if(poses.size() == 0) cout<<"camino vacío"<<endl;

    double d, _d;
    d = poses[0].position.z; // Esta es la distancia a la que el robot se encuentra del obstáculo más cercano

    // Quitar todas las posiciones que están "dentro del robot"
    while(poses.size())
        if(hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y) <= robotRadius)
            poses.erase(poses.begin());
        else break;

    //if(poses.size() == 0) cout<<"final de camino"<<endl;

    int i = 0;
    while(poses.size() > 1){
        _d = hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y);
        if(poses[1].position.z > poses[0].position.z && _d <= maxLenght){ // Si me estoy alejando de los obstáculos
            poses.erase(poses.begin());
        }else{
            return poses[0];
        }
    }

    return poses[0];
}

geometry_msgs::Pose PathFollower::selGoal_distance(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght)
{
    geometry_msgs::Pose res;

    double d, _d;
    d = poses[0].position.z; // Esta es la distancia a la que el robot se encuentra del obstáculo más cercano

    // Encuentro el punto más cercano del camino al robot
    int ind = -1;
    d = INFINITY;
    for(int i=0; i<poses.size(); i++){
        _d = hypot(pose.position.x - poses[i].position.x, pose.position.y - poses[i].position.y);
        if(_d < d){
            d = _d;
            ind = i;
        }
    }
    
    if(ind > -1){
        // Elimino todos los puntos hasta el punto más cercano
        poses.erase(poses.begin() + ind);
        // en el caso de estar dentro del camino, elimino todos los puntos que están "dentro del robot"
        while(poses.size())
            if(hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y) <= robotRadius)
                poses.erase(poses.begin());
            else break;

    }

    int i = 0;
    while(poses.size() > 1){
        _d = hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y);
        if(poses[1].position.z > poses[0].position.z && _d <= maxLenght){ // Si me estoy alejando de los obstáculos
            poses.erase(poses.begin());
        }else{
            return poses[0];
        }
    }

    return poses[0];
}

geometry_msgs::Pose PathFollower::selGoal_distances(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght, double &dist, double &dist2goal)
{
    // - pose: posición del robot
    // - poses: poses del camino
    // - maxLenght: distancia que se considera para alcanzar el goal
    // - dist: distancia mínima normalizada del camino al obstáculo más cercano
    // - dist2goal: distancia real desde punto del camino seleccionado al obstáculo

    geometry_msgs::Pose res;

    double d, _d;

    dist = 0;

    // Encuentro el punto más cercano del camino al robot
    int ind = -1;
    d = INFINITY;
    for(int i=0; i<poses.size(); i++){
        _d = hypot(pose.position.x - poses[i].position.x, pose.position.y - poses[i].position.y);
        if(_d < d){
            d = _d;
            ind = i;
        }
        if(dist < poses[i].position.z) dist = poses[i].position.z;
    }

    double dmax = dist;
    if(ind > -1){
        // Elimino todos los puntos hasta el punto más cercano
        //poses.erase(poses.begin() + ind);
        poses.erase(poses.begin(), poses.begin() + ind);
        // en el caso de estar dentro del camino, elimino todos los puntos que están "dentro del robot"
        while(poses.size())
            if(hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y) <= robotRadius){
                if(dist < poses[0].position.z) dist = poses[0].position.z;
                poses.erase(poses.begin());
            }else break;

    }

    dist2goal = INFINITY;

    int i = 0;
    while(poses.size() > 1){
        _d = hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y);
        if(dist2goal > poses[0].position.z) dist2goal = poses[0].position.z; // Almacenar la distancia al obstáculo
        if(poses[1].position.z > poses[0].position.z && _d <= maxLenght){ // Si me estoy alejando de los obstáculos
            if(dist < poses[0].position.z) dist = poses[0].position.z;
            poses.erase(poses.begin());
        }else{
            dist /= dmax;
            return poses[0];
        }
    }

    dist /= dmax;

    return poses[0];
}

geometry_msgs::Pose PathFollower::goalAndVel(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght)
{
    // Estoy suponiendo que el agente se encuentra sobre el camino (en la primera posición)
    geometry_msgs::Pose res;
    //cout<<"DISTANCIA MÁXIMA: "<<maxLenght<<endl;

    double angleDiff = std::numeric_limits<double>::max();
    double minObst = std::numeric_limits<double>::max();
    double minDist = std::numeric_limits<double>::max();
    double _d;
    vector<double> angles(poses.size(), std::numeric_limits<double>::max());
    vector<double> dist(poses.size(), std::numeric_limits<double>::max());
    for(int i=0; i<poses.size(); i++){
        // Distancia al punto del camino
        dist[i] = sqrt(pow(pose.position.x - poses[i].position.x, 2) + pow(pose.position.y - poses[i].position.y, 2));
        // Ángulo al punto del camino
        angles[i] = angle(robotPose, poses[i]);

        // Almacenar los valores mínimos
        if(minObst > poses[i].position.z) minObst = poses[i].position.z;
        _d = poses[i].position.z - robotRadius;
        if(minObst > _d) minObst = _d;
        if(minDist > dist[i]) minDist = dist[i];
        if(abs(angleDiff) > abs(angles[i])) angleDiff = angles[i];

        //cout<<"CALCULOS: "<<poses[i].position.z<<" ("<<minObst<<") - "<<dist[i]<<"("<<minDist<<") - "<<angles[i]<<" ("<<angleDiff<<")"<<endl;
        cout<<"CALCULOS: "<<_d<<" ("<<minObst<<") - "<<dist[i]<<"("<<minDist<<") - "<<angles[i]<<" ("<<angleDiff<<")"<<endl;
    }
    //cin.get();

    // Eliminar puntos del camino (que ya se pueden considerar alcanzados)
    while(poses.size()){
        if(sqrt(pow(pose.position.x - poses[0].position.x, 2) + pow(pose.position.y - poses[0].position.y, 2)) <= maxLenght){
            poses.erase(poses.begin());
            angles.erase(angles.begin());
            dist.erase(dist.begin());
        }else{
            //cout<<"SELECCIONADO: "<<poses[0].position.z<<" - "<<dist[0]<<" - "<<angles[0]<<endl;
            return poses[0];
        }
    }

    return res;
}


// *********************************************************************************************************************************
// 							FUNCIONES PARA SEGUIR CAMINO SIMPLE (SIN TENER EN CUENTA EL ENTORNO)
// *********************************************************************************************************************************


void PathFollower::initializeVariables(double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double goalDistDiff, double goalAngleDiff)
{
	this->maxLinearVel = maxLinearVel;
    this->maxAngularVel = maxAngularVel;
    this->linearAcc = linearAcc;
	this->angularAcc = angularAcc;
	this->goalDistDiff = goalDistDiff;
	this->goalAngleDiff = goalAngleDiff;

    cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
    _cmd_vel = cmd_vel;
}

void PathFollower::initializeTopics(string robotFrame, string robotPoseTopic, string robotCmdVelTopic)
{
	//poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::robotPoseCallback, this);
    if(robotPoseTopic.find("amcl")>=0 && robotPoseTopic.find("amcl")<robotPoseTopic.size()-1){
        // Subscriber pose con AMCL
        poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::amclPoseCallback, this);
    }else if((robotPoseTopic.find("odom")>=0 && robotPoseTopic.find("odom")<robotPoseTopic.size()-1) || (robotPoseTopic.find("ground_truth")>=0 && robotPoseTopic.find("ground_truth")<robotPoseTopic.size()-1)){
        // Subscriber pose con odometría
        poseSub = n_.subscribe<nav_msgs::Odometry>(robotPoseTopic, 1, &PathFollower::odomPoseCallback, this);
    }

	cmdVelPub = n_.advertise<geometry_msgs::Twist>(robotCmdVelTopic, 1);

    totalPathPub = n_.advertise<nav_msgs::Path>("totalPath", 10);

    baseFrame = robotFrame;
}

void PathFollower::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
	robotPose = robotPoseMsg->pose.pose;
	poseCh = true;

    geometry_msgs::PoseStamped _p; _p.pose = robotPoseMsg->pose.pose;
    totalPath.poses.push_back(_p);
    totalPath.header.frame_id = "map";
    totalPath.header.stamp = ros::Time::now();
    totalPathPub.publish(totalPath); // Publico
}

void PathFollower::odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    robotPose = robotPoseMsg->pose.pose;
    poseCh = true;

    geometry_msgs::PoseStamped _p; _p.pose = robotPoseMsg->pose.pose;
    totalPath.poses.push_back(_p);
    totalPath.header.frame_id = "map";
    totalPath.header.stamp = ros::Time::now();
    totalPathPub.publish(totalPath); // Publico
}

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic) 
{
	//n_ = nh;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
}// Constructor para seguimiento simple

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, double maxLinearVel, double maxAngularVel, double posDiff, double angleDiff)
{
	//n_ = nh;
	robotPose.position.x = -100000;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
	initializeVariables(maxLinearVel, maxAngularVel, linearAcc, angularAcc, posDiff, angleDiff);
}// Constructor para seguimiento simple

void PathFollower::computeVelocitiesSimple(const geometry_msgs::Pose goalPose, double& linearVel, double& angularVel)
{
    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular
    linearVel = maxLinearVel;
    angularVel = maxAngularVel * angleDiff;
} // no se tiene en cuenta nada

void PathFollower::computeVelocitiesAcc(const geometry_msgs::Pose goalPose, double dObstMaxNorm, double& linearVel, double& angularVel)
{
    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular (ajustado con las aceleraciones lineal y angular máximas)
    angularVel = maxAngularVel * angleDiff;
    if(angularVel < 0) 
        angularVel = max(angularVel, _cmd_vel.angular.z - abs(angularAcc));
    else
        angularVel = min(angularVel, _cmd_vel.angular.z + abs(angularAcc));

    if(angleDiff){
        //linearVel = maxLinearVel * goalPose.position.z * abs(maxAngularVel - abs(angularVel));
        if(abs(angleDiff) < M_PI/2)
            linearVel = maxLinearVel * dObstMaxNorm * abs(maxAngularVel - abs(angularVel));
        else
            linearVel = 0;
        //linearVel = maxLinearVel * abs(maxAngularVel - abs(angularVel));
        linearVel = min(linearVel, _cmd_vel.linear.x + linearAcc);
    }else{
        //linearVel = maxLinearVel;
        linearVel = min(maxLinearVel, _cmd_vel.linear.x + linearAcc);
    }

} // Se tiene en cuenta el ángulo al goal y las aceleraciones del robot


// *********************************************************************************************************************************
// 										FUNCIONES PARA SEGUIR CAMINO CON DWA
// *********************************************************************************************************************************

void PathFollower::initializeVariables(double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double goalDistDiff, double goalAngleDiff, double angularResolution, double maxObstacleDistance)
{
	this->maxLinearVel = maxLinearVel;
    this->maxAngularVel = maxAngularVel;
    this->linearAcc= linearAcc;
	this->angularAcc = angularAcc;
	this->goalDistDiff = goalDistDiff;
	this->goalAngleDiff = goalAngleDiff;
	this->angularResolution = angularResolution;
	this->maxObstacleDistance = maxObstacleDistance;
}

void PathFollower::initializeTopics(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, string laserTopic)
{
	//poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::robotPoseCallback, this);
    if(robotPoseTopic.find("amcl")>=0 && robotPoseTopic.find("amcl")<=robotPoseTopic.size()-1){
        // Subscriber pose con AMCL
        poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::amclPoseCallback, this);
    }else if(robotPoseTopic.find("odom")>=0 && robotPoseTopic.find("odom")<=robotPoseTopic.size()-1){
        // Subscriber pose con odometría
        poseSub = n_.subscribe<nav_msgs::Odometry>(robotPoseTopic, 1, &PathFollower::odomPoseCallback, this);
    }
	cmdVelPub = n_.advertise<geometry_msgs::Twist>(robotCmdVelTopic, 1);
	laserSub = n_.subscribe<sensor_msgs::LaserScan>(laserTopic, 1, &PathFollower::laserCallback, this);
    baseFrame = robotFrame;
}

void PathFollower::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg)
{
	this->laserData = *laserScanMsg;
	laserCh = true;
}

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, string laserTopic)
{
	//n_ = nh;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic, laserTopic);
	dwaPlanner = true;
}// Constructor para seguimiento con DWA

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double goalDistDiff, double goalAngleDiff, double angularResolution, double maxObstacleDistance)
{
	//n_ = nh;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic, laserTopic);
	initializeVariables(maxLinearVel, maxAngularVel, goalDistDiff, goalAngleDiff, angularResolution, maxObstacleDistance);
	dwaPlanner = true;
} // Constructor para seguimiento con DWA

void PathFollower::computeVelocitiesDWA(const geometry_msgs::Pose goalPose, double& linearVel, double& angularVel)
{

    // Inicialización de las velocidades
    linearVel = 0.0;
    angularVel = 0.0;

    // Cálculo del ángulo hacia el punto objetivo
    dx = goalPose.position.x - robotPose.position.x;
    dy = goalPose.position.y - robotPose.position.y;
    targetAngle = atan2(dy, dx);

    // Ajuste del ángulo hacia el punto objetivo en el rango [-pi, pi]
    if (targetAngle > M_PI)
        targetAngle -= 2.0 * M_PI;
    else if (targetAngle < -M_PI)
        targetAngle += 2.0 * M_PI;

    // Búsqueda de la mejor velocidad lineal y angular dentro de la ventana dinámica
    bestLinearVel = 0.0;
    bestAngularVel = 0.0;
    minCost = std::numeric_limits<double>::max();

    double _linearVel;
    for (double _angularVel = -maxAngularVel; _angularVel <= maxAngularVel; _angularVel += angularResolution)
    {
        // Cálculo de la velocidad lineal correspondiente al radio de curvatura
        _linearVel = std::min(maxLinearVel, std::abs(_angularVel) * robotRadius);

        //cout<<_linearVel<<" | "<<_angularVel<<endl;

        // Cálculo del costo de la velocidad actual
        cost = 0.0;

        // Simulación de movimiento para calcular el costo
        for (double t = 0.0; t <= 1.0; t += 0.1)
        {

            // Posición y orientación simuladas
            double simX = robotPose.position.x + _linearVel * cos(robotPose.orientation.z + _angularVel * t);
            double simY = robotPose.position.y + _linearVel * sin(robotPose.orientation.z + _angularVel * t);

            // Cálculo de la distancia al objetivo
            double distanceToGoal = std::hypot(simX - goalPose.position.x, simY - goalPose.position.y);

            // Cálculo del costo como una combinación de distancia al objetivo y proximidad a obstáculos
            double obstacleCost = 0.0;
            for (size_t i = 0; i < laserData.ranges.size(); ++i)
            {
                double angle = laserData.angle_min + i * laserData.angle_increment;
                double obstacleDistance = laserData.ranges[i];

                // Ignorar lecturas de láser inválidas o demasiado distantes
                if (std::isnan(obstacleDistance) || obstacleDistance > maxObstacleDistance)
                    continue;

                // Calcular proximidad a obstáculos y sumar al costo
                double obstacleAngle = angle + robotPose.orientation.z + _angularVel * t;
                double obstacleX = robotPose.position.x + obstacleDistance * cos(obstacleAngle);
                double obstacleY = robotPose.position.y + obstacleDistance * sin(obstacleAngle);
                double distanceToObstacle = std::hypot(obstacleX - simX, obstacleY - simY);
                obstacleCost += 1.0 / distanceToObstacle;
            }

            // Costo total de la velocidad actual
            double totalCost = distanceToGoal + obstacleCost;

            //cout<<t<<": ("<<simX<<", "<<simY<<"): "<<distanceToGoal<<" ---> coste: "<<totalCost<<" | "<<minCost<<endl;
            //cout<<t<<": ("<<robotPose.position.x<<", "<<robotPose.position.y<<", "<<robotPose.orientation.z<<") ---> ("<<simX<<", "<<simY<<") con coste "<<totalCost<<endl;

            // Actualizar la mejor velocidad y costo
            if (totalCost < minCost)
            {
                minCost = totalCost;
                bestLinearVel = _linearVel;
                bestAngularVel = _angularVel;
            }

            //cout<<"SELECCIÓN: ("<<bestLinearVel<<", "<<bestAngularVel<<"): "<<minCost<<endl;

        }
    }

    // Asignación de las velocidades resultantes
    linearVel = bestLinearVel;
    angularVel = bestAngularVel;
}

// *********************************************************************************************************************************

bool PathFollower::followPath(nav_msgs::Path path)
{
    ROS_INFO("SIGUIENDO CAMINO (path) ... ");
    if(dwaPlanner) followPathDWA(path);
    if(path.poses.size() == 0) return 0;
    while(!poseCh){ // Esperar a leer la posición del robot
        //cout<<"Position  : ("<<robotPose.position.x<<", "<<robotPose.position.y<<", "<<robotPose.position.z<<")"<<endl;
        //cout<<"Quaternion: ("<<robotPose.orientation.x<<", "<<robotPose.orientation.y<<", "<<robotPose.orientation.z<<", "<<robotPose.orientation.w<<")"<<endl;
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
    this->path = path;
    
    while(ros::ok() && path.poses.size()){
        // Publicar el footprint del robot
        footprintPub.publish(computeFootprint(robotPose, sX, sY));
        // Obtener el siguiente punto del camino a alcanzar y "alcualizar" el camino
        goalPose = nextGoal(robotPose, path.poses, goalDistDiff);
        // Calcular (v,w)
        computeVelocitiesSimple(goalPose, cmd_vel.linear.x, cmd_vel.angular.z);
        //computeVelocities(goalPose, cmd_vel.linear.x, cmd_vel.angular.z);
        // Publicar la trayectoría que sigue el robot
        planPub.publish(setPlan(robotPose, goalPose));
        // Publicar (v,w)
        cmdVelPub.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(sleepTime).sleep();
    }

    ROS_INFO("publico (%f, %f)",cmd_vel.linear.x, cmd_vel.angular.z);

    return true;
} // Sigue el camino

bool PathFollower::followPath(vector<geometry_msgs::Pose> path)
{
    //ROS_INFO("SIGUIENDO CAMINO (poses) ... ");
	if(dwaPlanner) followPathDWA(path);
	if(path.size() == 0) return 0;
	while(!poseCh){ // Esperar a leer la posición del robot
		//cout<<"Position  : ("<<robotPose.position.x<<", "<<robotPose.position.y<<", "<<robotPose.position.z<<")"<<endl;
        //cout<<"Quaternion: ("<<robotPose.orientation.x<<", "<<robotPose.orientation.y<<", "<<robotPose.orientation.z<<", "<<robotPose.orientation.w<<")"<<endl;
        ROS_INFO("ESPERANDO POSICION DEL ROBOT ... ");
        ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	
	pathPoses = path;

    ros::Publisher textPub = n_.advertise<visualization_msgs::Marker>("texto", 10);
    visualization_msgs::Marker texto;
    texto.id = 0;
    texto.header.seq = 0;
    texto.header.stamp = ros::Time::now();
    texto.header.frame_id = baseFrame;
    texto.action = visualization_msgs::Marker::ADD;
    texto.pose.orientation.w = 1;
    texto.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    texto.ns = "texto";
    texto.scale.x = texto.scale.y = texto.scale.z = 0.15;
    texto.color.b = 1.0; texto.color.a = 1.0;
	
	while(ros::ok()){
        // Publicar el footprint del robot
        footprintPub.publish(computeFootprint(robotPose, sX, sY));
        if(pathPoses.size() > 1){

            double d, d2g;
            /*
            int ind;
            ind = findClosest(robotPose, path);
            if(ind > -1 && path[ind].position.z) d = path[ind].position.z;
            else d = 1;
            */

            //cout<<"calculo pose y velocidad: "<<pathPoses.size()<<endl;
            // Obtener el siguiente punto del camino a alcanzar y "alcualizar" el camino
            //goalPose = goalAndVel(robotPose, pathPoses, goalDist);
            //goalPose = selGoal(robotPose, pathPoses, goalDist);
            int npa = pathPoses.size();
            goalPose = selGoal_distances(robotPose, pathPoses, goalDist, d, d2g);

            int npd = pathPoses.size();

            //cout<<"tamaño del camino fuera: "<<pathPoses.size()<<endl;

            //cout<<"Distancia al obstáculo: "<<d2g<< " | "<<robotRadius<<endl;
            //cout<<"Distancias: "<<d<< " - "<<d2g<<" | "<<robotRadius<<endl;

    		// Calcular (v,w)
            //computeVelocitiesSimple(goalPose, cmd_vel.linear.x, cmd_vel.angular.z);
            if(pathPoses.size()){
                if(d2g > 2*robotRadius) computeVelocitiesAcc(goalPose, 1, cmd_vel.linear.x, cmd_vel.angular.z);
                else computeVelocitiesAcc(goalPose, d, cmd_vel.linear.x, cmd_vel.angular.z);
            }else cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
            
            //cmd_vel.linear.x = -1.0, cmd_vel.angular.z = 0;

            _cmd_vel = cmd_vel; // Almacenar las velocidades anteriores
            // Publicar la trayectoría que sigue el robot
            planPub.publish(setPlan(robotPose, goalPose));
            //texto.text = "camino: "+to_string(npa)+" - "+to_string(npd);
            texto.text = "camino: "+to_string(npa-npd);
            textPub.publish(texto);

            //cout<<"publico velocidades ("<<cmd_vel.linear.x<<", "<<cmd_vel.angular.z<<")"<<endl;
            //cout<<"--------------------------------------"<<endl;
        }else{
            cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        }

        // Publicar (v,w)
        cmdVelPub.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(sleepTime).sleep(); // tiempo en segundos
	}

    ROS_INFO("publico (%f, %f)",cmd_vel.linear.x, cmd_vel.angular.z);

	return true;
} // Sigue el camino

bool PathFollower::followPath(vector<geometry_msgs::PoseStamped> path)
{
    pathPoses.clear();
	for(int i=0; i<path.size(); i++)
		pathPoses.push_back(path[i].pose);
	return followPath(pathPoses);
} // Sigue el camino


bool PathFollower::followPathDWA(vector<geometry_msgs::Pose> path)
{
    ROS_INFO("SIGUIENDO CAMINO CON DWA (POSES) ... ");
	if(path.size() == 0) return 0;
	while(!poseCh && !laserCh){ // Esperar a leer la posición del robot y el láser
		//cout<<"Position  : ("<<robotPose.position.x<<", "<<robotPose.position.y<<", "<<robotPose.position.z<<")"<<endl;
        //cout<<"Quaternion: ("<<robotPose.orientation.x<<", "<<robotPose.orientation.y<<", "<<robotPose.orientation.z<<", "<<robotPose.orientation.w<<")"<<endl;
        ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
    ROS_INFO("POSE Y LASER RECIBIDOS");
	
	pathPoses = path;
	
	while(ros::ok() && pathPoses.size()){
        // Obtener el siguiente punto del camino a alcanzar y "altualizar" el camino
        goalPose = nextGoal(robotPose, pathPoses, goalDistDiff);
		// Cálculo de (v,w)
        //ROS_INFO("calculo velocidades para (%f, %f) ---> (%f, %f)", robotPose.position.x, robotPose.position.y, pathPoses[0].position.x, pathPoses[0].position.y);
		computeVelocitiesDWA(goalPose, cmd_vel.linear.x, cmd_vel.angular.z);
        // Publicar la trayectoría que sigue el robot
        planPub.publish(setPlan(robotPose, goalPose));
        // Publicar (v,w)
        cmdVelPub.publish(cmd_vel);
        // Publicar el footprint del robot
        footprintPub.publish(computeFootprint(robotPose, sX, sY));
        ros::spinOnce();
        ros::Duration(sleepTime).sleep();
	}

	return true;
} // Sigue el camino

bool PathFollower::followPathDWA(nav_msgs::Path path)
{
    ROS_INFO("SIGUIENDO CAMINO CON DWA (CAMINO) ... ");
    if(path.poses.size() == 0) return 0;
    while(!poseCh && !laserCh){ // Esperar a leer la posición del robot y el láser
        //cout<<"Position  : ("<<robotPose.position.x<<", "<<robotPose.position.y<<", "<<robotPose.position.z<<")"<<endl;
        //cout<<"Quaternion: ("<<robotPose.orientation.x<<", "<<robotPose.orientation.y<<", "<<robotPose.orientation.z<<", "<<robotPose.orientation.w<<")"<<endl;
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("POSE Y LASER RECIBIDOS");
    
    this->path = path;
    
    while(ros::ok() && path.poses.size()){
        // Obtener el siguiente punto del camino a alcanzar y "alcualizar" el camino
        goalPose = nextGoal(robotPose, path.poses, goalDistDiff);
        // Cálculo de (v,w)
        //ROS_INFO("calculo velocidades para (%f, %f) ---> (%f, %f)", robotPose.position.x, robotPose.position.y, pathPoses[0].position.x, pathPoses[0].position.y);
        computeVelocitiesDWA(goalPose, cmd_vel.linear.x, cmd_vel.angular.z);
        // Publicar la trayectoría que sigue el robot
        planPub.publish(setPlan(robotPose, goalPose));
        // Publicar (v,w)
        cmdVelPub.publish(cmd_vel);
        // Publicar el footprint del robot
        footprintPub.publish(computeFootprint(robotPose, sX, sY));
        ros::spinOnce();
        ros::Duration(sleepTime).sleep();
    }

    return true;
} // Sigue el camino
