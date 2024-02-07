
#include <multi_dynamic/multi_util.h>
#include <multi_dynamic/util/funciones.hpp>
#include <multi_dynamic/util/fmm_2.hpp>
#include <multi_dynamic/util/generic_fmm.hpp>
#include <multi_dynamic/util/geometry.hpp>
#include <multi_dynamic/util/graph_functions.hpp>

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
int navAreaType = 0; // 0: libre, 1: segmentos, 2: areas circulares

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
fmm_segment::Segments fmm_segments;

vector<vector<float>> refGrad, obstGrad;
vector<Poss<int>> gridPositions;
vector<vector<float>> navArea; // Espacio por el que navegan los obstáculos

vector<geometry_msgs::Point> navAreasPoints;

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

vector<geometry_msgs::Point> poss2Points(Poss<int> poss, const nav_msgs::OccupancyGrid& map)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    for(int j=0; j<poss.x.size(); j++){
        p.y = poss.x[j] * map.info.resolution + map.info.origin.position.x;
        p.x = poss.y[j] * map.info.resolution + map.info.origin.position.y;
        res.push_back(p);
    }
    return res;
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
    // Posiciones sobre el mapa
    double x = 0;
    double y = 0;
    double r = 0;

    geometry_msgs::Point gridPoint;

    void setGridPoint(const nav_msgs::OccupancyGrid& mapMsg)
    {
        gridPoint.y = (x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
        gridPoint.x = (y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
        gridPoint.z = r / mapMsg.info.resolution;
    }

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
            //_area.center.x = _area.x; _area.center.y = _area.y;
            res.push_back(_area);
        }else if(*_argv[i]==';'){
            cout<<"Termina área"<<endl;
            i++; // Me pungo en la siguiente "línea"
            // Creo nueva área
            _area.x = atof(_argv[i]); i++;
            _area.y = atof(_argv[i]); i++;
            _area.r = atof(_argv[i]); //i++;
            // Guardar en el punto
            //_area.gr.x = _area.x; _area.center.y = _area.y; _area.center.z = _area.r;
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

Poss<int> getNeighbourPoss(int seg, vector<vector<bool>> graph, vector<Poss<int>> poss)
{
    Poss<int> res;

    if(seg < 0) return res;

    //res.append(poss[seg]);

    for(int i=0; i<graph[seg].size(); i++){
        if(graph[seg][i]){
            res.append(poss[i]);
        }
    }

    return res;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "obstacles");
	ros::NodeHandle nh;
    ros::Rate loopRate(100);

    vector<area> dynamicAreas;

	if(argc >= 12){
        map_topic = argv[1];
        footPrint_topic = argv[2];
        Nobst = atoi(argv[3]);
        sX = atof(argv[4]); sY = atof(argv[5]);
        maxLinearVel = atof(argv[6]);
        maxAngularVel = atof(argv[7]);
        linearAcc = atof(argv[8]);
        angularAcc = atof(argv[9]);
        seed = atoi(argv[10]);
        navAreaType = atoi(argv[11]);
        if(navAreaType == 2)
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

    radius = 1.5*hypot(sX, sY);

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

        fmm_segments = fmm_segment::compute_segments(grid, obstGrad);

        cout<<"Segmentos obtenidos"<<endl;
    }

    // Fijar el espacio en el que navegan los obstáculos
    if(navAreaType == 0){ // Libre
        navArea = obstGrad;
    }else if(navAreaType == 1){ // Segmentos
        navArea.resize(gsx,vector<float>(gsy, 0));
        currPosesGrid = multi_util::poses2GridCoordinates(currPoses, mapMsg); // Puntos sobre el grid
        vector<bool> nav_segs(fmm_segments.contour_poss.size(), false);
        Poss<int> seg_poss;
        for(int i=0; i<currPosesGrid.size(); i++){
            // Obtener el segmento en el que se encuentra el obstáculo
            int seg = fmm_segments.map[currPosesGrid[i].position.y][currPosesGrid[i].position.x];
            // Fijar el área del segmento como área navegable
            //cout<<"OBSTÁCULO "<<i<<": "<<obstGrad[currPosesGrid[i].position.x][currPosesGrid[i].position.y]<<" en segmento "<<seg<<endl;
            if(seg >= 0 && !nav_segs[seg]){
                //nav_segs[seg] = true;
                vector<int> neighbours = graph_functions::neighbors(fmm_segments.graph, seg);
                //sh_vect_h(neighbours, "vecinos");
                //seg_poss = getNeighbourPoss(seg, fmm_segments.graph, fmm_segments.poss);
                //seg_poss = fmm_segments.poss[seg];
                for(int j=0; j<neighbours.size(); j++){
                    if(nav_segs[neighbours[j]]) continue;
                    nav_segs[neighbours[j]] = true;
                    // Fijar los puntos del área como navegables
                    seg_poss = fmm_segments.poss[neighbours[j]];
                    for(int k=0; k<seg_poss.x.size(); k++){
                        navArea[seg_poss.x[k]][seg_poss.y[k]] = obstGrad[seg_poss.x[k]][seg_poss.y[k]];
                    }
                    
                    // Almacenar los contornos de las áreas en las que se encuentran los obstáculos
                    Poss<int> _poss = geometry::sort_points(fmm_segments.centroids.x[neighbours[j]], fmm_segments.centroids.y[neighbours[j]], fmm_segments.contour_poss[neighbours[j]], 1);
                    _poss.push(_poss.x[0], _poss.y[0]); // Para cerrar el círculo
                    vector<geometry_msgs::Point> _p = poss2Points(_poss, mapMsg);
                    navAreasPoints.insert(navAreasPoints.end(), _p.begin(), _p.end());
                    
                }

                /*
                // ERROR QUE ME DA
                Uninitialized quaternion, assuming identity.
                Number of points should be a multiple of 2 for LINE_LIST marker.
                */

                // Almacenar los contornos de las áreas en las que se encuentran los obstáculos
                Poss<int> _poss = geometry::sort_points(fmm_segments.centroids.x[seg], fmm_segments.centroids.y[seg], fmm_segments.contour_poss[seg], 1);
                _poss.push(_poss.x[0], _poss.y[0]);
                vector<geometry_msgs::Point> _p = poss2Points(_poss, mapMsg);
                //vector<geometry_msgs::Point> _p = poss2Points(fmm_segments.contour_poss[seg], mapMsg);
                navAreasPoints.insert(navAreasPoints.end(), _p.begin(), _p.end());
            }
        }
    }else if(navAreaType == 2){ // Áreas circulares        
        navArea.resize(gsx,vector<float>(gsy, 0));
        for(int k=0; k<dynamicAreas.size(); k++){
            vector<geometry_msgs::Point> _p = multi_util::circlePoints(dynamicAreas[k].x, dynamicAreas[k].y, dynamicAreas[k].r, 0.1);
            navAreasPoints.insert(navAreasPoints.end(), _p.begin(), _p.end());
            // Obtener el área sobre el grid
            dynamicAreas[k].setGridPoint(mapMsg);
            //cout<<"area "<<k<<": ("<<dynamicAreas[k].gridPoint.x<<", "<<dynamicAreas[k].gridPoint.y<<", "<<dynamicAreas[k].gridPoint.z<<")"<<endl;
            // Fijar las posiciones del área como navegables
            for(int i=0; i<navArea.size(); i++){
                for(int j=0; j<navArea[i].size(); j++){
                    if(hypot(dynamicAreas[k].gridPoint.x - i, dynamicAreas[k].gridPoint.y - j) <= dynamicAreas[k].gridPoint.z){
                        navArea[i][j] = obstGrad[i][j];
                    }
                }
            }
        }
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
    cmd_vel_pat[1].linear.x = maxLinearVel;
    cmd_vel_pat[1].angular.z = 0.0;
    Tlb[1] = 5.0; Tub[1] = 10.0;
    // Giro
    cmd_vel_pat[2].linear.x = 0.0;
    cmd_vel_pat[2].angular.z = maxAngularVel;
    Tlb[2] = 1.0; Tub[2] = 3.0;
    // Giro
    cmd_vel_pat[3].linear.x = 0.0;
    cmd_vel_pat[3].angular.z = -maxAngularVel;
    Tlb[3] = 1.0; Tub[3] = 3.0;

    // Guardar el área navegable de los obstáculos
    save_matr("obstNavArea.txt", navArea);

	int cont = 0;
	while(ros::ok()){

        currPosesGrid = multi_util::poses2GridCoordinates(currPoses, mapMsg);

        beginT = ros::Time::now();

        // Limpiar los marcadores anteriores de los footprints
		obstFootprintMarkers.points.clear(); obstFootprintMarkers.colors.clear();

        // Las posiciones de los obstáculos que se van a utilizar para chequear la "posibilidad de movimiento"
        virtualPoses = currPoses;
        _currPoses = currPoses;

		// Mover los obstáculos
		for(int i=0; i<Nobst; i++){

            // Generar movimiento aleatorio
            if(Tmov[i] <= 0){
                // Generar el movimiento
                indPat = patSel(gen);
                while((indPat == 0 && actions[i] == 0) || (actions[i] < 2 && stuck[i] && indPat < 2)){ // El obstáculo está quieto o se ha quedado estancado
                    // Fuerzo movimiento (tras estar quieto) o girar (tras estar estancado)
                    indPat = patSel(gen);
                }
                // Generar duración del movimiento
                std::uniform_real_distribution<double> timeGen(Tlb[indPat], Tub[indPat]);
                Tmov[i] = timeGen(gen);
                // Fijar las velocidades correspondientes al movimiento
                cmd_vel[i] = cmd_vel_pat[indPat];
                // Almacenar el movimiento seleccionado
                actions[i] = indPat;
            }

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

            // Obtener la posición del obstáculo sobre el grid
            multi_util::toGridCoordinates(goalPoses[i].position.y, goalPoses[i].position.x, mapMsg, goalPosesGrid[i].position.x, goalPosesGrid[i].position.y);
            //cout<<"obstáculo "<<i<<" esta a "<<navArea[goalPosesGrid[i].position.x][goalPosesGrid[i].position.y]<<" > "<<radiusGrid/2<<" y "<<!checkCollission(goalPoses[i], radius/2, i, virtualPoses)<<endl;

            // Publicar las velocidades
            //if(obstGrad[goalPosesGrid[i].position.x][goalPosesGrid[i].position.y] > radiusGrid/2 && !checkCollission(goalPoses[i], radius/2, i, virtualPoses) && checkAreas(goalPoses[i], dynamicAreas)){
            if((navArea[goalPosesGrid[i].position.x][goalPosesGrid[i].position.y] > radiusGrid/2) && !checkCollission(goalPoses[i], radius/2, i, virtualPoses)){
                //cout<<"publico velocidad"<<endl;
                // Movimiento viable
                cmd_vel_pub[i].publish(cmd_vel[i]);
                virtualPoses[i] = goalPoses[i];
                stuck[i] = false;
            }else{
                //cout<<"NO VIABLE"<<endl;
                // Movimiento no viable
                cmd_vel[i] = cmd_vel_pat[0];
                cmd_vel_pub[i].publish(cmd_vel[i]);
                stuck[i] = true; // Obstáculo atascado
            }

            //cout<<i<<" velocidad publicada: "<<cmd_vel[i].linear.x<<", "<<cmd_vel[i].angular.z<<endl;

            velocitiesVec[i].x = goalPoses[i].position.x - currPoses[i].position.x;
            velocitiesVec[i].y = goalPoses[i].position.y - currPoses[i].position.y;
		}

        /*
        for(int i=0; i<dynamicAreas.size(); i++){
            vector<geometry_msgs::Point> _p = multi_util::circlePoints(dynamicAreas[i].x, dynamicAreas[i].y, dynamicAreas[i].r, 0.1);
            obstFootprintMarkers.points.insert(obstFootprintMarkers.points.end(), _p.begin(), _p.end());
        }
        */

        // Añadir las áreas dinámicas para representar
        if(navAreasPoints.size()){
            obstFootprintMarkers.points.insert(obstFootprintMarkers.points.end(), navAreasPoints.begin(), navAreasPoints.end());
            //cout<<obstFootprintMarkers.points.size()<<" puntos de las áreas"<<endl;
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