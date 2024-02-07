
#include <multi_dynamic/multi_util.h>
#include <multi_dynamic/util/funciones.hpp>
#include <multi_dynamic/util/fmm_2.hpp>
#include <multi_dynamic/util/generic_fmm.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <random>

// Parámetros de entrada de los obstáculos
string map_topic = "map", footPrint_topic = "obstFootprint";
int Nobst = 0;
double sX = 0.25;
double sY = 0.45;
double radius = hypot(sX, sY);
double radiusGrid = 0.0;
double maxLinearVel = 0.55;
double maxAngularVel = 1;
double linearAcc = 0.05;
double angularAcc = 0.05;
int seed;

vector<geometry_msgs::Pose> currPoses, goalPoses, virtualPoses; // Posiciones sobre el mapa
vector<geometry_msgs::Pose> _currPoses; // Posiciones previas sobre el mapa
vector<geometry_msgs::Pose> currPosesGrid, goalPosesGrid; // Posiciones sobre el grid (por tanto tambien sobre gradiente)
vector<geometry_msgs::Point> velocitiesVec;
vector<vector<geometry_msgs::Point>> currPoints, goalPoints;
vector<vector<geometry_msgs::Point>> currPointsGrid, goalPointsGrid;
vector<ros::Subscriber> pose_sub;
vector<ros::Publisher> cmd_vel_pub;
ros::Publisher cmd_vel_lines_pub;
ros::Publisher footprints_pub;
ros::Publisher areas_pub;
visualization_msgs::Marker obstFootprintMarkers;
vector<visualization_msgs::Marker> areaMarkers;

// VAriables del mapa
nav_msgs::OccupancyGrid mapMsg;
vector<vector<int>> occGrid;
vector<vector<float>> grid;
int gsx, gsy;

vector<vector<float>> refGrad, obstGrad;
vector<Poss<int>> gridPositions;

int _Nobst = 0;

vector<geometry_msgs::Twist> cmd_vel; // (v, w) actuales de los obstáculos
vector<geometry_msgs::Twist> _cmd_vel; // (v, w) anteriores de los obstáculos

// Variables que se utilizan para los patrones de movimiento
float Twalk = 3.0;
float Tstop = 7.0;
float Tmeet = 10.0;
float Twork = 15.0;

float Rint = 1.5;
float Rmeet = 1.5;


vector<geometry_msgs::Pose> destinations, destinationsGrid;

vector<int> actions;
vector<bool> stuck;
vector<float> timer;

std::random_device rd;
//std::mt19937 gen(rd());

std::uniform_int_distribution<int> actionGen(0, 4);

void poseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    currPoses[_Nobst] = robotPoseMsg->pose.pose;
}

void _poseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg, geometry_msgs::Pose& pose)
{
    pose = robotPoseMsg->pose.pose;
}

geometry_msgs::TwistStamped setCmdVel(geometry_msgs::Twist vel, string frame)
{
    geometry_msgs::TwistStamped res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = frame;
    res.twist = vel;
    return res; 
}

geometry_msgs::Pose nextPose(geometry_msgs::Pose pose, geometry_msgs::Twist cmdVel)
{

    // Calcular la nueva pose del robot después de la simulación
    double theta = tf::getYaw(pose.orientation);

    // Actualizar la pose del robot usando las velocidades lineales y angulares
    pose.position.x += cmdVel.linear.x * cos(theta);
    pose.position.y += cmdVel.linear.x * sin(theta);
    theta += cmdVel.angular.z;

    pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    return pose;
}

struct area{
    double x = 0;
    double y = 0;
    double r = 0;

    geometry_msgs::Point center;

    void show(){ cout<<"("<<x<<", "<<y<<", "<<r<<")"<<endl; }
};

vector<area> readAreas(int _argc, char** _argv)
{
    vector<area> res;
    int i = 0;

    vector<double> data;
    area _area;
    string s;
    while(i < _argc){
        if(*_argv[i]=='['){
            cout<<"Empiezan áreas"<<endl;
            // Creo nueva área
            i++; // Me pungo en la siguiente "línea"
            _area.x = atof(_argv[i]); i++;
            _area.y = atof(_argv[i]); i++;
            _area.r = atof(_argv[i]); //i++;
            // Guardar en el punto
            _area.center.x = _area.x; _area.center.y = _area.y;
            res.push_back(_area);
        }else if(*_argv[i]==';'){
            cout<<"Termina área"<<endl;
            i++; // Me pungo en la siguiente "línea"
            // Creo nueva área
            _area.x = atof(_argv[i]); i++;
            _area.y = atof(_argv[i]); i++;
            _area.r = atof(_argv[i]); //i++;
            // Guardar en el punto
            _area.center.x = _area.x; _area.center.y = _area.y;
            res.push_back(_area);
        }else if(*_argv[i]==']'){
            return res;
        }else{
            i++;
        }
    }
    return res;
}

bool checkCollission(geometry_msgs::Pose pose, double r, int ind, vector<geometry_msgs::Pose> poses)
{
    for(int i=0; i<poses.size(); i++){
        if(i!=ind && hypot(pose.position.x - poses[i].position.x, pose.position.y - poses[i].position.y) <= r){
            //cout<<ind<<" choca con "<<i<<endl;
            return true;
        }
    }
    return false;
}


vector<bool> checkPositions(vector<geometry_msgs::Pose> poses, vector<area> areas)
{
    vector<bool> res(poses.size(), true);
    for(int i=0; i<poses.size(); i++){
        if(checkCollission(poses[i], radius, i, poses)){
            res[i] = false;
        }else{
            for(int j=0; j<areas.size(); j++){
                if(hypot(poses[i].position.x - areas[j].x, poses[i].position.y - areas[j].y) > areas[i].r){
                    res[i] = false;
                    break;
                }
            }
        }
    }
    return res;   
}

bool checkAreas(geometry_msgs::Pose pose, vector<area> areas)
{
    for(int j=0; j<areas.size(); j++){
        if(hypot(pose.position.x - areas[j].x, pose.position.y - areas[j].y) < areas[j].r){
            // Estoy dentro de un área
            return true;
        }
    }
    return false;
}

bool checkPosition(int ind, vector<geometry_msgs::Pose> poses, vector<area> areas)
{
    if(checkCollission(poses[ind], radius, ind, poses)){
        cout<<"colisiono con "<<ind<<endl;
        return false;
    }
    if(!checkAreas(poses[ind], areas)) return false;
    
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "obstacles");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    vector<area> dynamicAreas;

    if(argc >= 11){
        map_topic = argv[1];
        footPrint_topic = argv[2];
        Nobst = atoi(argv[3]);
        sX = atof(argv[4]); sY = atof(argv[5]);
        maxLinearVel = atof(argv[6]);
        maxAngularVel = atof(argv[7]);
        linearAcc = atof(argv[8]);
        angularAcc = atof(argv[9]);
        seed = atoi(argv[10]);
        dynamicAreas = readAreas(argc, argv);
    }

    std::mt19937 gen;
    if(seed) gen.seed(seed);
    else gen.seed(rd());

    cout<<argc<<" argumentos"<<endl;
    cout<<"map topic:   "<<map_topic<<endl;
    cout<<"foot topic:  "<<footPrint_topic<<endl;
    cout<<"N obstacles: "<<Nobst<<endl;
    cout<<"tamaño:      "<<sX<<" x "<<sY<<endl;

    cout<<"linear vel:  "<<maxLinearVel<<endl;
    cout<<"angular vel: "<<maxAngularVel<<endl;
    cout<<"linear acc:  "<<linearAcc<<endl;
    cout<<"angular acc: "<<angularAcc<<endl;
    cout<<"semilla:     "<<seed<<endl;

    if(dynamicAreas.size()){
        cout<<dynamicAreas.size()<<" areas"<<endl;
        for(int i=0; i<dynamicAreas.size(); i++) dynamicAreas[i].show();
    }

    radius = 1.5*hypot(sX, sY);

    /*
    for(int i=0; i<argc; i++)
        cout<<argv[i]<<endl;
    */

    // -----------------------------------------------------------------------------------------

    currPoses.resize(Nobst);
    goalPoses.resize(Nobst);
    cmd_vel.resize(Nobst);
    _cmd_vel.resize(Nobst);
    currPoints.resize(Nobst);
    goalPoints.resize(Nobst);
    velocitiesVec.resize(Nobst);

    vector<bool> move(Nobst, true);

    _Nobst = 0;
    string robot_name, robot_str;
    for(int i=1; i<=Nobst; i++){
        robot_name = "robot_" + to_string(i);
        robot_str = tf::resolve(robot_name, "cmd_vel");
        cout<<robot_str<<endl;
        cmd_vel_pub.push_back(nh.advertise<geometry_msgs::Twist>(robot_str, 10));
        robot_str = tf::resolve(robot_name, "base_pose_ground_truth");
        cout<<robot_str<<endl;
        //pose_sub.push_back(nh.subscribe<nav_msgs::Odometry>(robot_str, 10, &poseCallback));
        //nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, boost::bind(multi_util::laserPosCallback, _1, boost::ref(laserPos)))
        pose_sub.push_back(nh.subscribe<nav_msgs::Odometry>(robot_str, 10, boost::bind(_poseCallback, _1, boost::ref(currPoses[_Nobst]))));

        while(ros::ok() && currPoses[_Nobst].position.x == 0){
            // Esperar hasta recibir las posiciones de los obstáculos
            ros::spinOnce();  // Procesar los callbacks pendientes
            ros::Duration(0.01).sleep();  // Esperar un breve periodo de tiempo
        }

        _Nobst++;
    }

    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(multi_util::gridCallback, _1, boost::ref(mapMsg), boost::ref(occGrid), boost::ref(grid)));
    while(!grid.size()){
        // Esperar hasta recibir las posiciones de los obstáculos
        ros::spinOnce();  // Procesar los callbacks pendientes
        ros::Duration(0.01).sleep();  // Esperar un breve periodo de tiempo
    }

    goalPoses = currPoses;
    currPosesGrid = multi_util::poses2GridCoordinates(currPoses, mapMsg);
    goalPosesGrid = currPosesGrid;

    radiusGrid = radius/mapMsg.info.resolution;

    gsx = grid.size();
    gsy = grid[0].size();

    ros::Time ahora, despues;
    ahora = ros::Time::now();
    ros::Duration(0.12).sleep();
    despues = ros::Time::now();
    float tdiff = despues.toSec() - ahora.toSec();
    cout<<tdiff<<endl;

    if(grid.size()){

        // Calculo el gradiente desde la posición del agente para obtener el espacio libre en el que se va a mover
        FMM gr(currPosesGrid[0].position.y, currPosesGrid[0].position.x, grid);
        refGrad = gr.compute_gradient_();

        // Cargarme las posiciones "no alcanzables" del grid
        gridPositions = fix_free_space(grid, refGrad);

        // Gradiente de los obstáculos
        FMM ogr(gridPositions[1].x, gridPositions[1].y, grid);
        obstGrad = ogr.compute_gradient_();

        cout<<"Gradientes obtenidos"<<endl;
    }

    // Inicializar el publisher para pintar los footprints de los obstáculos
    footprints_pub = nh.advertise<visualization_msgs::Marker>("obstFootprints", 1);
    
    obstFootprintMarkers.header.frame_id = map_topic;
    obstFootprintMarkers.id = 0;
    obstFootprintMarkers.ns = "obstacles_footprints";
    obstFootprintMarkers.action = visualization_msgs::Marker::ADD;
    obstFootprintMarkers.type = visualization_msgs::Marker::LINE_LIST;
    obstFootprintMarkers.lifetime = ros::Duration(1.0); // El marcador durará 1 segundo en RViz
    obstFootprintMarkers.scale.x = 0.02;

    vector<geometry_msgs::Point> _points;

    // MOVIMIENTO ALEATORIO DE LOS OBSTÁCULOS DINÁMICOS
    
    std::uniform_real_distribution<double> linDist(0, maxLinearVel);
    std::uniform_real_distribution<double> angDist(-maxAngularVel, maxAngularVel);
    
    // Para los patrones de movimiento
    actions.resize(Nobst,0); // Todos quietos al principio
    ros::Time beginT, endT; // Tiempos
    timer.resize(Nobst, 0);
    stuck.resize(Nobst, false);

    vector<float> Tmov(Nobst, 0);

    int indPat;
    vector<geometry_msgs::Twist> cmd_vel_pat(4);
    vector<float> Tlb(4), Tub(4);
    std::uniform_int_distribution<int> patSel(0, 3);
    vector<string> patStr = {"quieto", "recto", "giro izq", "giro der"};
    
    // Quieto
    cmd_vel_pat[0].linear.x = 0.0;
    cmd_vel_pat[0].angular.z = 0.0;
    Tlb[0] = 5.0; Tub[0] = 15.0;
    // Recto
    cmd_vel_pat[1].linear.x = maxLinearVel/3;
    cmd_vel_pat[1].angular.z = 0.0;
    Tlb[1] = 5.0; Tub[1] = 10.0;
    // Giro
    cmd_vel_pat[2].linear.x = 0.0;
    cmd_vel_pat[2].angular.z = maxAngularVel/2;
    Tlb[2] = 1.0; Tub[2] = 3.0;
    // Giro
    cmd_vel_pat[3].linear.x = 0.0;
    cmd_vel_pat[3].angular.z = -maxAngularVel/2;
    Tlb[3] = 1.0; Tub[3] = 3.0;

    int cont = 0;
    while(ros::ok()){

        beginT = ros::Time::now();

        // Limpiar los marcadores anteriores de los footprints
        obstFootprintMarkers.points.clear(); obstFootprintMarkers.colors.clear();

        // Las posiciones de los obstáculos que se van a utilizar para chequear la "posibilidad de movimiento"
        virtualPoses = currPoses;
        _currPoses = currPoses;

        // Mover los obstáculos
        for(int i=0; i<Nobst; i++){

            //// Asignar velocidades constantes
            //cmd_vel[i].linear.x = maxLinearVel/3;
            //cmd_vel[i].angular.z = maxAngularVel;

            //// Generar velocidades aleatorias
            //cmd_vel[i].linear.x = linDist(gen);
            //cmd_vel[i].angular.z = angDist(gen);

            // Generar movimiento aleatorio
            if(Tmov[i] <= 0){
                indPat = patSel(gen);
                // Generar duración del movimiento
                std::uniform_real_distribution<double> timeGen(Tlb[indPat], Tub[indPat]);
                Tmov[i] = timeGen(gen);
                // Fijar las velocidades correspondientes al movimiento
                while((indPat == 0 && actions[i] == 0) || (actions[i] < 2 && stuck[i] && indPat < 2)){ // El obstáculo está quieto o se ha quedado estancado
                    // Fuerzo movimiento (tras estar quieto) o girar (tras estar estancado)
                    indPat = patSel(gen);
                }
                cmd_vel[i] = cmd_vel_pat[indPat];
                //cout<<"Agente "<<i<<" "<<patStr[indPat]<<" ("<<cmd_vel[i].linear.x<<", "<<cmd_vel[i].angular.z<<"): "<<Tmov[i]<<endl;

                actions[i] = indPat;
            }

            //cout<<cmd_vel[i].linear.x<<" - "<<cmd_vel[i].angular.z<<endl;

            // Puntos de la dirección que lleva el obstáculo
            goalPoses[i] = nextPose(currPoses[i], cmd_vel[i]);
            // Almacenar los puntos para representar
            obstFootprintMarkers.points.push_back(currPoses[i].position);
            obstFootprintMarkers.points.push_back(goalPoses[i].position);

            // Posición actual del obstáculo
            currPoints[i] = multi_util::footprint(currPoses[i], sX, sY);
            // Posición a la que se va a mover el obstáculo
            goalPoints[i] = multi_util::footprint(goalPoses[i], sX, sY);
            // Almacenar los puntos para representar
            obstFootprintMarkers.points.insert(obstFootprintMarkers.points.end(), currPoints[i].begin(), currPoints[i].end());
            //obstFootprintMarkers.points.insert(obstFootprintMarkers.points.end(), goalPoints[i].begin(), goalPoints[i].end());

            //cout<<"Obstáculo "<<i<<" ("<<currPoses[i].position.x<<", "<<currPoses[i].position.y<<"): "<<obstGrad[currPosesGrid[i].position.y][currPosesGrid[i].position.x]<<" | "<<radiusGrid<<endl;

            // Obtener la posición del obstáculo sobre el grid
            multi_util::toGridCoordinates(goalPoses[i].position.y, goalPoses[i].position.x, mapMsg, goalPosesGrid[i].position.x, goalPosesGrid[i].position.y);
            // Publicar las velocidades
            if(obstGrad[goalPosesGrid[i].position.x][goalPosesGrid[i].position.y] > radiusGrid/2 && !checkCollission(goalPoses[i], radius/2, i, virtualPoses) && checkAreas(goalPoses[i], dynamicAreas)){
                // Movimiento viable
                //cout<<"obstáculo "<<i<<" esta a "<<obstGrad[goalPosesGrid[i].position.x][goalPosesGrid[i].position.y]<<" > "<<radiusGrid<<endl;
                cmd_vel_pub[i].publish(cmd_vel[i]);
                virtualPoses[i] = goalPoses[i];
                stuck[i] = false;
            }else{
                // Movimiento no viable
                cmd_vel[i] = cmd_vel_pat[0];
                cmd_vel_pub[i].publish(cmd_vel[i]);
                stuck[i] = true; // Obstáculo atascado
            }

            velocitiesVec[i].x = goalPoses[i].position.x - currPoses[i].position.x;
            velocitiesVec[i].y = goalPoses[i].position.y - currPoses[i].position.y;
        }

        for(int i=0; i<dynamicAreas.size(); i++){
            vector<geometry_msgs::Point> _p = multi_util::circlePoints(dynamicAreas[i].x, dynamicAreas[i].y, dynamicAreas[i].r, 0.1);
            obstFootprintMarkers.points.insert(obstFootprintMarkers.points.end(), _p.begin(), _p.end());
        }

        //obstFootprintMarkers.color.r = 1.0; obstFootprintMarkers.color.a = 1.0;
        obstFootprintMarkers.color.r = 0.1; obstFootprintMarkers.color.b = 0.1; obstFootprintMarkers.color.g = 0.1; obstFootprintMarkers.color.a = 1.0;
        footprints_pub.publish(obstFootprintMarkers);

        ros::spinOnce();
        loopRate.sleep();

        endT = ros::Time::now();

        // Actualizar los tiempos
        for(int i=0; i<Nobst; i++){
            timer[i] -= (endT.toSec() - beginT.toSec());
            Tmov[i] -= (endT.toSec() - beginT.toSec());
        }

        cont++;
    }

    return 0;
}