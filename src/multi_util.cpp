
#include <multi_dynamic/multi_util.h>


double multi_util::length(geometry_msgs::Point p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

double multi_util::length(geometry_msgs::Twist v)
{
    return sqrt(v.linear.x * v.linear.x + v.angular.z * v.angular.z);
}

double multi_util::lengthSquared(geometry_msgs::Point p)
{
    return p.x * p.x + p.y * p.y + p.z * p.z;
}

geometry_msgs::Point multi_util::normalize(geometry_msgs::Point p)
{
    double len = length(p);
    if (len == 0) return p;

    p.x /= len;
    p.y /= len;
    p.z /= len;

    return p;
}

geometry_msgs::Twist multi_util::normalize(geometry_msgs::Twist v)
{
    double len = length(v);
    if (len == 0) return v;

    v.linear.x /= len;
    v.angular.z /= len;

    return v;
}

geometry_msgs::Point multi_util::leftNormalVector(geometry_msgs::Point p)
{
    geometry_msgs::Point res;
    res.x = -p.y;
    res.y = p.x;
    return res;
}

geometry_msgs::Point multi_util::rightNormalVector(geometry_msgs::Point p)
{
    geometry_msgs::Point res;
    res.x = p.y;
    res.y = -p.x;
    return res;
}

double multi_util::angleTo(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return multi_util::polarAngle(p2) - multi_util::polarAngle(p1);
}

double multi_util::normalizeAngle(double angle)
{
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

double multi_util::polarAngle(geometry_msgs::Point p)
{
    return normalizeAngle(atan2(p.y, p.x));
}

int multi_util::sign(double value)
{
    if (value == 0)
        return 0;
    else if (value > 0)
        return 1;
    else
        return -1;
}

// ******************************************************************************************************************************
//                                                    Posiciones en el mapa
// ******************************************************************************************************************************

geometry_msgs::Pose multi_util::getRobotPose()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Pose currPose;

    try {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(1.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

        currPose.position.x = transform.getOrigin().x(); // <----------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        currPose.position.y = transform.getOrigin().y(); // <----------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        currPose.position.z = transform.getOrigin().z();

        currPose.orientation.x = transform.getRotation().x();
        currPose.orientation.y = transform.getRotation().y();
        currPose.orientation.z = transform.getRotation().z();
        currPose.orientation.w = transform.getRotation().w();

        cout<<"POSICIÓN DEL ROBOT: ("<<currPose.position.x<<", "<<currPose.position.y<<")"<<endl;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return currPose;
}

pair<double, double> multi_util::getRobotPosition()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<double, double> multi_util::getRobotPosition(int robot_no)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
    	string robot_str = "/robot_";
    	robot_str += boost::lexical_cast<string>(robot_no);
    	string base_footprint_frame = tf::resolve(robot_str, "base_footprint");

        listener.waitForTransform("/map", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", base_footprint_frame, ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<double, double> multi_util::getRobotMapPosition(int robot_no) // ????????????????????????????????????????????????????????????????''
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
        string robot_str = "/robot_";
        robot_str += boost::lexical_cast<string>(robot_no);
        string base_footprint_frame = tf::resolve(robot_str, "base_footprint");

        listener.waitForTransform(base_footprint_frame, "/map", ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform(base_footprint_frame, "/map", ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<int, int> multi_util::transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> robotPos)
{
  // Calcular la posición del robot en coordenadas del grid
  pair<int, int> res;
  res.second = (robotPos.first - map.info.origin.position.x) / map.info.resolution;
  res.first = (robotPos.second - map.info.origin.position.y) / map.info.resolution;
  return res;
}

pair<int, int> multi_util::transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, geometry_msgs::Pose pose)
{
  // Calcular la posición del robot en coordenadas del grid
  pair<int, int> res;
  res.second = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
  res.first = (pose.position.y - map.info.origin.position.y) / map.info.resolution;
  return res;
}

geometry_msgs::Point multi_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, double x, double y)
{
    geometry_msgs::Point res;
    res.x = (x - map.info.origin.position.x) / map.info.resolution;
    res.y = (y - map.info.origin.position.y) / map.info.resolution;
    return res;
}

geometry_msgs::Point multi_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> pos)
{
    geometry_msgs::Point res;
    res.x = (pos.first - map.info.origin.position.x) / map.info.resolution;
    res.y = (pos.second - map.info.origin.position.y) / map.info.resolution;
    return res;
}


vector<geometry_msgs::Point> multi_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::Point> points(poses.size());
    for(int i=0; i<poses.size(); i++){
        points[i].x = (poses[i].position.x - map.info.origin.position.x) / map.info.resolution;
        points[i].y = (poses[i].position.y - map.info.origin.position.y) / map.info.resolution;
    }
    return points;
}

void multi_util::toGridCoordinates(double x, double y, const nav_msgs::OccupancyGrid& map, double &gx, double &gy)
{
    gx = (x - map.info.origin.position.x) / map.info.resolution;
    gy = (y - map.info.origin.position.y) / map.info.resolution;
}

vector<geometry_msgs::Pose> multi_util::poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map)
{
    for(int i=0; i<poses.size(); i++){
        poses[i].position.x = (poses[i].position.x - map.info.origin.position.x) / map.info.resolution;
        poses[i].position.y = (poses[i].position.y - map.info.origin.position.y) / map.info.resolution;
    }
    return poses;
}

vector<geometry_msgs::Point> multi_util::points2GridCoordinates(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map)
{
    for(int i=0; i<points.size(); i++){
        points[i].x = (points[i].x - map.info.origin.position.x) / map.info.resolution;
        points[i].y = (points[i].y - map.info.origin.position.y) / map.info.resolution;
    }
    return points;
}

vector<pair<double, double>> multi_util::getTeamPositions(int team_size)
{
	vector<pair<double, double>> currPositions(team_size);
	for (int i = 0; i < team_size; i++) {
		currPositions[i] = multi_util::getRobotPosition(i);
	}
	return currPositions;
}

geometry_msgs::Pose multi_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<int, int> robotGridPos)
{
    // Calcular la posición del robot en el mapa a partir de sus coordenadas del grid
    geometry_msgs::Pose res;
    res.position.x = robotGridPos.first * map.info.resolution + map.info.origin.position.x;
    res.position.y = robotGridPos.second * map.info.resolution + map.info.origin.position.y;
    res.position.z = 0;

    res.orientation.w = 1.0;
    return res;
}

geometry_msgs::Pose multi_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, int x, int y)
{
    // Calcular la posición del robot en el mapa a partir de sus coordenadas del grid
    geometry_msgs::Pose res;
    res.position.x = x * map.info.resolution + map.info.origin.position.x;
    res.position.y = y * map.info.resolution + map.info.origin.position.y;
    res.position.z = 0;

    res.orientation.w = 1.0;

    return res;
}

geometry_msgs::Point multi_util::tfGridPos2MapPoint(const nav_msgs::OccupancyGrid& map, int x, int y)
{
    geometry_msgs::Point res;
        res.x = x * map.info.resolution + map.info.origin.position.x;
        res.y = y * map.info.resolution + map.info.origin.position.y;
        res.z = 0;
    return res;
}

vector<geometry_msgs::Point> multi_util::tfGridPos2MapPos(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y)
{
    vector<geometry_msgs::Point> res(x.size());
    for(int i=0; i<res.size(); i++){
        res[i].x = x[i] * map.info.resolution + map.info.origin.position.x;
        res[i].y = y[i] * map.info.resolution + map.info.origin.position.y;
        res[i].z = 0;
    }
    return res;
}

vector<geometry_msgs::PoseStamped> multi_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y)
{
    vector<geometry_msgs::PoseStamped> res(x.size());
    for(int i=0; i<res.size(); i++){
        res[i].pose.position.x = x[i] * map.info.resolution + map.info.origin.position.x;
        res[i].pose.position.y = y[i] * map.info.resolution + map.info.origin.position.y;
        res[i].pose.position.z = 0;
        // Aqui debería calcular la orientación
    }
    return res;
}

vector<geometry_msgs::PoseStamped> multi_util::poses2PoseStamped(vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::PoseStamped> res(poses.size());
    for(int i=0; i<res.size(); i++){
        res[i].pose = poses[i];
    }
    return res;
}

vector<geometry_msgs::Point> multi_util::buildGraphPoints(vector<int> x, vector<int> y, vector<vector<bool>> graph, nav_msgs::OccupancyGrid map)
{
    vector<geometry_msgs::Point> points;
    for(int i=0; i<graph.size(); i++){
        for(int j=i+1; j<graph[i].size(); j++){
            if(graph[i][j]){
                points.push_back(multi_util::tfGridPos2MapPoint(map, x[i], y[i]));
                points.push_back(multi_util::tfGridPos2MapPoint(map, x[j], y[j]));
            }
        }
    }
    return points;
}

vector<geometry_msgs::Point> multi_util::buildTreePoints(vector<int> x, vector<int> y, vector<vector<int>> tree, nav_msgs::OccupancyGrid map)
{
    vector<geometry_msgs::Point> points;
    for(int i=0; i<tree.size(); i++){
        points.push_back(multi_util::tfGridPos2MapPoint(map, x[tree[i][0]], y[tree[i][0]]));
        points.push_back(multi_util::tfGridPos2MapPoint(map, x[tree[i][1]], y[tree[i][1]]));
    }
    return points;
}

vector<geometry_msgs::Point> multi_util::linkPoints(vector<geometry_msgs::Point> points, vector<vector<bool>> graph)
{
    vector<geometry_msgs::Point> res;
    for(int i=0; i<graph.size(); i++){
        for(int j=i+1; j<graph[i].size(); j++){
            if(graph[i][j]){
                res.push_back(points[i]);
                res.push_back(points[j]);
            }
        }
    }
    return res;
}

vector<geometry_msgs::Point> multi_util::linkPoints(vector<geometry_msgs::Point> points, vector<vector<int>> tree)
{
    vector<geometry_msgs::Point> res;
    for(int i=0; i<tree.size(); i++){
        res.push_back(points[tree[i][0]]);
        res.push_back(points[tree[i][1]]);
    }
    return res;
}

vector<geometry_msgs::Point> multi_util::linkPoints(vector<geometry_msgs::Point> points, vector<int> branch)
{
    vector<geometry_msgs::Point> res;
    if(branch.size())
        for(int i=0; i<branch.size()-1; i++){
            res.push_back(points[branch[i]]);
            res.push_back(points[branch[i+1]]);
        }
    return res;
}

geometry_msgs::Point32 multi_util::rotatePoint(geometry_msgs::Pose pose, double x, double y)
{
    geometry_msgs::Point32 p;
    double th = tf::getYaw(pose.orientation);
    double s = sin(th);
    double c = cos(th);
    p.x = c * (x - p.x) - s * (y - p.y) + x;
    p.y = s * (x - p.x) + c * (y - p.y) + y;
    return p;
}

vector<geometry_msgs::Point> multi_util::footprint(geometry_msgs::Pose pose, double sx, double sy)
{
    vector<geometry_msgs::Point> res;

    // Calcular los vértices del footprint como un rectángulo
    std::vector<geometry_msgs::Point> vertices;
    double half_sX = sx / 2;
    double half_sY = sy / 2;

    // Calcular los vértices del rectángulo no rotado
    tf::Vector3 p1(half_sX, half_sY, 0);
    tf::Vector3 p2(half_sX, -half_sY, 0);
    tf::Vector3 p3(-half_sX, -half_sY, 0);
    tf::Vector3 p4(-half_sX, half_sY, 0);

    // Rotar los vértices según la orientación del robot
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    p1 = tf::quatRotate(q, p1);
    p2 = tf::quatRotate(q, p2);
    p3 = tf::quatRotate(q, p3);
    p4 = tf::quatRotate(q, p4);

    // Trasladar los vértices según la posición del robot
    p1 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p2 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p3 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p4 += tf::Vector3(pose.position.x, pose.position.y, 0);

    // Convertir a geometry_msgs::Point y agregar al vector de puntos del marcador
    geometry_msgs::Point gp1, gp2, gp3, gp4;
    tf::pointTFToMsg(p1, gp1);
    tf::pointTFToMsg(p2, gp2);
    tf::pointTFToMsg(p3, gp3);
    tf::pointTFToMsg(p4, gp4);

    res.push_back(gp1);
    res.push_back(gp2);
    res.push_back(gp2);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp4);
    res.push_back(gp4);
    res.push_back(gp1);

    return res;
}

vector<geometry_msgs::Point> multi_util::circlePoints(double x, double y, double r, double rs)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p1, p2;

    for (double angle = 0; angle < 2 * M_PI; angle += rs)
    {
        p1.x = x + r * cos(angle);
        p1.y = y + r * sin(angle);
        p1.z = 0.0;

        angle += 0.1;
        p2.x = x + r * cos(angle);
        p2.y = y + r * sin(angle);
        p2.z = 0.0;

        res.push_back(p1);
        res.push_back(p2);
    }
    return res;
}

bool multi_util::checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float d, float ad)
{
    pair<float, float> dist = poseDistance(p1,p2);
    if(dist.first<=d && dist.second<=ad) return true;
    return false;
}

pair<float, float> multi_util::poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    pair<float, float> res;
    res.first = sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
    float o1 = tf::getYaw(p1.orientation);
    float o2 = tf::getYaw(p2.orientation);
    res.second = abs(atan2(sin(o1-o2), cos(o1-o2)));
    return res;
}

float multi_util::pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float multi_util::orientationDist(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
    float o1 = tf::getYaw(q1);
    float o2 = tf::getYaw(q2);
    return abs(atan2(sin(o1-o2), cos(o1-o2)));
}

// ******************************************************************************************************************************
//                                                         Laser
// ******************************************************************************************************************************


void multi_util::_laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos){
    //ROS_INFO("=====================> LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
    laserPos.clear();
    // Obtener los parámetros del láser scan
    float angle_min = laser_scan->angle_min;
    float angle_max = laser_scan->angle_max;
    float angle_increment = laser_scan->angle_increment;
    float range_max = laser_scan->range_max;
    float range_min = laser_scan->range_min;

    // Calcular la cantidad de muestras en el láser scan
    int num_samples = (angle_max - angle_min) / angle_increment;

    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < num_samples; ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        float angle = angle_min + i * angle_increment;

        // Obtener la distancia medida por el láser en la muestra actual
        float range = laser_scan->ranges[i];

        // Verificar si la distancia medida está dentro del rango válido
        if (range >= range_min && range <= range_max){
            // Calcular la posición en el grid
            float x = range * cos(angle);
            float y = range * sin(angle);

            // Crear un punto de geometría con la posición en el grid
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Agregar la posición al vector de posiciones en el grid
            laserPos.push_back(position);
        }
    }
    //cout<<laserPos.size()<<" posiciones"<<endl;
}

void multi_util::laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos){
    //ROS_INFO("=====================> LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
    laserPos.clear();

    float angle;

    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < laser_scan->ranges.size(); ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        angle = laser_scan->angle_min + i * laser_scan->angle_increment;

        // Verificar si la distancia medida está dentro del rango válido
        if (laser_scan->ranges[i] >= laser_scan->range_min && laser_scan->ranges[i] <= laser_scan->range_max){
            // Calcular la posición en el grid
            float x = laser_scan->ranges[i] * cos(angle);
            float y = laser_scan->ranges[i] * sin(angle);

            // Crear un punto de geometría con la posición en el grid
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Agregar la posición al vector de posiciones en el grid
            laserPos.push_back(position);
        }
    }
    //cout<<laserPos.size()<<" posiciones"<<endl;
}

void multi_util::laserPosesCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Pose>& laserPoses)
{
    laserPoses.clear();

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
            pose.position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Creo el quaternion
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            // Agregar la posición al vector de posiciones en el grid
            laserPoses.push_back(pose);
        }
    }
}

vector<geometry_msgs::Pose> multi_util::laserPosesTf(vector<geometry_msgs::Pose> laserPoses)
{
    tf::TransformListener tfListener;

    geometry_msgs::PoseStamped transformedPose, laserPose;
    vector<geometry_msgs::Pose> transformedPoses(laserPoses.size());
    for(int i=0; i<laserPoses.size(); i++){
        try {
            laserPose.header.frame_id = "/scan";
            laserPose.pose = laserPoses[i];
            // Realiza la transformación a través del marco de referencia global (map)
            tfListener.transformPose("/map", laserPose, transformedPose);

            // La pose transformada está ahora en transformedPose.pose
            // Puedes utilizar transformedPose.pose para representarla en tu grid de ocupación
            transformedPoses[i] = transformedPose.pose;
        } catch (tf::TransformException& ex) {
            ROS_WARN("Error en la transformacion del laser: %s", ex.what());
        }
    }
    return transformedPoses;
}

vector<geometry_msgs::Pose> multi_util::laserPosesTf(geometry_msgs::Pose robotPose, vector<geometry_msgs::Pose> laserPoses){

    // Realiza la transformación para cada pose del láser
    std::vector<geometry_msgs::Pose> transformedPoses;
    for (const auto& laserPose : laserPoses){
        // Aplica la transformación de la pose del láser al marco de referencia del robot
        geometry_msgs::Pose transformedPose;
        transformedPose.position.x = laserPose.position.x + robotPose.position.x;
        transformedPose.position.y = laserPose.position.y + robotPose.position.y;
        transformedPose.position.z = laserPose.position.z + robotPose.position.z;

        tf2::Quaternion q_robot(robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z, robotPose.orientation.w);
        tf2::Quaternion q_laser(laserPose.orientation.x, laserPose.orientation.y, laserPose.orientation.z, laserPose.orientation.w);

        tf2::Quaternion q_transformed = q_robot * q_laser;
        q_transformed.normalize();

        transformedPose.orientation.x = q_transformed.x();
        transformedPose.orientation.y = q_transformed.y();
        transformedPose.orientation.z = q_transformed.z();
        transformedPose.orientation.w = q_transformed.w();

        transformedPoses.push_back(transformedPose);
    }
    return transformedPoses;
}


// ******************************************************************************************************************************
//                                                         Mapa
// ******************************************************************************************************************************

void multi_util::_occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid)
{
    // Convertir el mensaje OccupancyGrid a un objeto vector<vector<int>>
    occGrid = occupancyGridToVector(*occupancyGridMsg);
}

void multi_util::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid)
{
    // Almacenar el mapa
    map = *occupancyGridMsg;
    // Y el grid
    occGrid = occupancyGridToVector(*occupancyGridMsg);
}

vector<vector<int>> multi_util::occupancyGridToVector(const nav_msgs::OccupancyGrid& occupancyGridMsg)
{
    vector<vector<int>> occGrid(occupancyGridMsg.info.height, vector<int>(occupancyGridMsg.info.width, 0));
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGridMsg.info.height; ++y){
        for (size_t x = 0; x < occupancyGridMsg.info.width; ++x){
            // Obtener el valor de ocupación de la celda
            occGrid[y][x] = occupancyGridMsg.data[y * occupancyGridMsg.info.width + x];
        }
    }

    return occGrid;
}

vector<vector<int>> multi_util::occupancyGrid2Grid(vector<vector<int>> occupancyGrid)
{
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGrid.size(); ++y){
        for (size_t x = 0; x < occupancyGrid[x].size(); ++x){
            if(occupancyGrid[y][x]>0) occupancyGrid[y][x] = 1;
            else occupancyGrid[y][x] = 0;
        }
    }

    return occupancyGrid;
}

void multi_util::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid, vector<vector<float>>& grid)
{
    // Almacenar el mapa
    map = *occupancyGridMsg;
    // Y los grids
    //occGrid = occupancyGridToVector(*occupancyGridMsg);
    //grid = occupancyGrid2Grid(occGrid);

    occupancyGrid2Grid(occupancyGridMsg, occGrid, grid);
}

void multi_util::occupancyGrid2Grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid, vector<vector<float>>& grid)
{
    occGrid.resize(occupancyGridMsg->info.height, vector<int>(occupancyGridMsg->info.width, 0));
    grid.resize(occupancyGridMsg->info.height, vector<float>(occupancyGridMsg->info.width, 0));
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGridMsg->info.height; ++y){
        for (size_t x = 0; x < occupancyGridMsg->info.width; ++x){
            // Obtener el valor de ocupación de la celda
            occGrid[y][x] = occupancyGridMsg->data[y * occupancyGridMsg->info.width + x];
            if(occGrid[y][x]==0) grid[y][x] = 1;
            else grid[y][x] = 0;
        }
    }
}

vector<signed char> multi_util::grid2occupancyGridData(vector<vector<float>> grid)
{
    vector<signed char> res;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j])
                res.push_back(0);
            else if(grid[i][j] == 0)
                res.push_back(100);
    return res;
}

vector<signed char> multi_util::grid2occupancyGridData(vector<vector<int>> grid)
{
    vector<signed char> res;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j])
                res.push_back(0);
            else if(grid[i][j] == 0)
                res.push_back(100);
    return res;
}

vector<signed char> multi_util::map2occupancyGridData(vector<vector<int>> map)
{
    vector<signed char> res;
    for(int i=0; i<map.size(); i++)
        for(int j=0; j<map[i].size(); j++)
            res.push_back(map[i][j]);
    return res;
}

vector<signed char> multi_util::map2occupancyGridData(vector<vector<float>> map)
{
    vector<signed char> res;
    for(int i=0; i<map.size(); i++)
        for(int j=0; j<map[i].size(); j++)
            res.push_back(map[i][j]);
    return res;
}


// ******************************************************************************************************************************
//                                               Planificación de caminos
// ******************************************************************************************************************************

vector<vector<float>> multi_util::navigableGradient(vector<vector<float>> obst_grad, float size)
{
    // Esto equivaldría a una especie de inflado de los obstáculos
    for(int i=0; i<obst_grad.size(); i++)
        for(int j=0; j<obst_grad[i].size(); j++)
            if(obst_grad[i][j] <= size) obst_grad[i][j] = 0;
    return obst_grad;
} // Dejo los valores del gradiente de los obstáculos → Los caminos se alejarán de los obstáculos

vector<vector<float>> multi_util::navigableGrid(vector<vector<float>> obst_grad, float size)
{
    for(int i=0; i<obst_grad.size(); i++)
        for(int j=0; j<obst_grad[i].size(); j++)
            if(obst_grad[i][j] <= size) obst_grad[i][j] = 0;
            else obst_grad[i][j] = 1;
    return obst_grad;
} // Los valores son [0,1] → Se obtienen los caminos más cortos evitando los obstáculos

nav_msgs::Path multi_util::transformToMapPath(vector<int> x, vector<int> y, const nav_msgs::OccupancyGrid& mapMsg)
{
    // POr si al final uso un navegador propio de ROS
    nav_msgs::Path res;
    geometry_msgs::PoseStamped currPose;
    for(int i=0; i<x.size(); i++){
        currPose.pose.position.x = x[i] * mapMsg.info.resolution + mapMsg.info.origin.position.x;
        currPose.pose.position.y = y[i] * mapMsg.info.resolution + mapMsg.info.origin.position.y;
        res.poses.push_back(currPose);
    }
    return res;
}

vector<geometry_msgs::Pose> multi_util::convertPointsToPoses(vector<geometry_msgs::Point> points)
{
    vector<geometry_msgs::Pose> poses;

    if (points.size() < 2){
        ROS_WARN("Not enough points in the path");
        return poses;
    }
    geometry_msgs::Pose pose;
    for (size_t i = 0; i < points.size() - 1; ++i){

        pose.position = points[i];

        // Calculate orientation (yaw) between current point and next point
        double dx = points[i + 1].x - points[i].x;
        double dy = points[i + 1].y - points[i].y;
        double orientation = atan2(dy, dx);

        // Convert orientation to quaternion representation
        pose.orientation = tf::createQuaternionMsgFromYaw(orientation);

        poses.push_back(pose);
    }

    // El último punto conserva la orientación del penúltimo para no girarte
    geometry_msgs::Pose lastPose;
    lastPose.position = points.back();
    lastPose.orientation  = pose.orientation;
    poses.push_back(lastPose);

    return poses;
}

geometry_msgs::Pose multi_util::computePose(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    geometry_msgs::Pose pose;
    pose.position = p1;
    pose.orientation = tf::createQuaternionMsgFromYaw(atan2(p2.y - p1.y, p2.x - p1.x));
    return pose;
}

float multi_util::computePathDistance(nav_msgs::Path path)
{
    float res = 0;

    if(path.poses.size() > 1)
    for(int i=1; i<path.poses.size(); i++)
        res += hypot(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x, path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);

    return res;
}

vector<geometry_msgs::Pose> multi_util::path2Poses(nav_msgs::Path path)
{
    vector<geometry_msgs::Pose> res;
    for(int i=0; i<path.poses.size(); i++)
        res.push_back(path.poses[i].pose);
    return res;   
}

pair<float, float> multi_util::distPose2Path(geometry_msgs::Pose p, nav_msgs::Path path, int evp)
{
    pair<float, float> res, d;
    res.first = res.second = 0;
    d.first = d.second = 0;
    int n = path.poses.size();
    if(evp >= n) evp = (n-1);
    //cout<<"ESTO: "<<n<<" - "<<evp<<endl;
    for(int i = n-1; i>=n-evp; i--){
        d = poseDistance(p, path.poses[i].pose);
        res.first += d.first;
        res.second += d.second;
        //cout<<i<<": ("<<p.position.x<<", "<<p.position.y<<") -> ("<<path.poses[i].pose.position.x<<", "<<path.poses[i].pose.position.y<<") = "<<d.first<<" | "<<d.second<<" = "<<res.first<<" | "<<res.second<<endl;
        //cout<<i<<": ("<<d.first<<", "<<d.second<<" | "<<pointDist(p.position, path.poses[i].pose.position)<<", "<<orientationDist(p.orientation, path.poses[i].pose.orientation)<<") - ("<<res.first<<", "<<res.second<<")"<<endl;
    }
    return res;
}

vector<geometry_msgs::Pose> multi_util::setFollowerPathPoses(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses)
{
    vector<geometry_msgs::Pose> path2Follow;
    // Obtener el camino que se le va a enviar al path follower
    // Pasar camino en el grid a puntos del mapa
    path2GoalPoses = multi_util::convertPointsToPoses(multi_util::tfGridPos2MapPos(mapMsg, y, x));

    // Obtener la distancia del camino a los obstáculos más cercanos
    vector<float> pathGradValues(x.size(), 0);
    path2Follow = path2GoalPoses;
    float maxv = -1;
    for(int i=0; i<x.size(); i++){
        pathGradValues[i] = navArea[x[i]][y[i]];
        if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
    }

    // Incluir las distancias de los obstáculos dinámicos en el camino
    float _d;
    for(int i=0; i<x.size(); i++){
        for(int j=0; j<vox.size(); j++){
            _d = hypot(vox[j] - x[i], voy[j] - y[i]);
            if(pathGradValues[i] < _d){
                pathGradValues[i] = _d;
            }
            if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
        }
    }

    // Normalizar los valores de las distancias del camino
    if(maxv)
        for(int i=0; i<x.size(); i++){
            //pathGradValues[i] /= maxv;
            //path2Follow[i].position.z = pathGradValues[i]; // Distancia sobre el grid
            path2Follow[i].position.z = pathGradValues[i] * mapMsg.info.resolution; // Distancia sobre el mapa
        }

    return path2Follow;
}


// ******************************************************************************************************************************
//                                                         Navegación
// ******************************************************************************************************************************

geometry_msgs::Twist multi_util::computeVelocities(pair<double, double> start, pair<double, double> goal)
{
    geometry_msgs::Twist res;

    double dist = sqrt(pow(start.first - goal.first, 2) + pow(start.second - goal.second, 2));

    //if(dist > MIN_DIST){
        res.linear.x = min(0.5 * dist, MAX_LINEAR_VEL);
        res.angular.z = min(4 * atan2(pow(start.second - goal.second, 2), pow(start.first - goal.first, 2)), MAX_ANGULAR_VEL);
    //}

    return res;
}

geometry_msgs::Twist multi_util::computeVelocities(pair<double, double> start, geometry_msgs::Point goal)
{
    geometry_msgs::Twist res;

    double dist = sqrt(pow(start.first - goal.x, 2) + pow(start.second - goal.y, 2));

    //if(dist > MIN_DIST){
        res.linear.x = min(0.5 * dist, MAX_LINEAR_VEL);
        res.angular.z = min(4 * atan2(pow(start.second - goal.y, 2), pow(start.first - goal.x, 2)), MAX_ANGULAR_VEL);
    //}

    return res;
}

// ******************************************************************************************************************************
//                                                         Representación
// ******************************************************************************************************************************

visualization_msgs::MarkerArray multi_util::setTreeLines(vector<geometry_msgs::Point> points, vector<vector<int>> tree, vector<float> colors, string map_topic, string tree_frame_id, float resolution)
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
        res.markers[i].scale.x = res.markers[i].scale.y = resolution;
        res.markers[i].color.b = colors[0];
        res.markers[i].color.r = colors[1];
        res.markers[i].color.g = colors[2];
        res.markers[i].color.a = 1.0;

        res.markers[i].points.push_back(points[tree[i][0]]);
        res.markers[i].points.push_back(points[tree[i][1]]);
    }
    return res;
}

void multi_util::clearTreeLines(visualization_msgs::MarkerArray &lines)
{
    for(int i=0; i<lines.markers.size(); i++){
        //lines.markers[i].points.clear();
        lines.markers[i].action = visualization_msgs::Marker::DELETEALL; 
    }
    lines.markers.clear();
}

// ******************************************************************************************************************************
//                                                         Basura
// ******************************************************************************************************************************

void multi_util::count(vector<vector<int>> grid)
{
    int obst, fs, us;
    obst = fs = us = 0;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j]==0){
                obst++;
            }else if(grid[i][j]>0){
                fs++;
            }else if(grid[i][j]<0){
                us++;
            }
    cout<<"Libre: "<<fs<<endl;
    cout<<"Obst : "<<obst<<endl;
    cout<<"Desc : "<<us<<endl;
}


