
// Seguir caminos
#include <multi_dynamic/path_follower.hpp>

PathFollower *pf;
vector<geometry_msgs::PoseStamped> path2GoalPoses;

void pathPosesCallback(const nav_msgs::Path pathMsg)
{
    //cout<<"ME HE SUSCRITO"<<endl;
	path2GoalPoses = pathMsg.poses;
	if(path2GoalPoses.size()){ // Se ha recibido el camino
		//ROS_INFO("CAMINO RECIBIDO");
		pf->followPath(path2GoalPoses); // Seguir el camino
	}else{ // NO se ha recibido el camino
		ROS_INFO("ESPERANDO CAMINO ..........");
	}
}

void pathCallback(const nav_msgs::Path pathMsg)
{
    cout<<"ME HE SUSCRITO"<<endl;
    if(pathMsg.poses.size()){ // Se ha recibido el camino
        ROS_INFO("CAMINO RECIBIDO");
        pf->followPath(pathMsg); // Seguir el camino
    }else{ // NO se ha recibido el camino
        ROS_INFO("ESPERANDO CAMINO ..........");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;

    // Obtener las variables con lo parámetros para seguir el camino
    string pathTopic;
    string robotFrame;
    string poseTopic, cmdVelTopic, laserTopic;
    string footprintTopic, planTopic; // Para representación
    double sX, sY; // Tamaño del robot
    double maxLinearVel, maxAngularVel, robotRadius;
    double linearAcc, angularAcc;
    double goalDistDiff, goalAngleDiff, angularResolution, maxObstacleDistance;
    string planner_type; // {'simple', 'dwa'}
    
    // args=[pathTopic robotPoseTopic sX sY robotCmdVelTopic laserTopic maxLinearVel maxAngularVel goalDistDiff goalAngleDiff |...
    // planner_type angularResolution maxObstacleDistance | footprintTopic planTopic]
    if(argc == 19){
        pathTopic = argv[1];
        robotFrame = argv[2];
        poseTopic = argv[3];
        sX = atof(argv[4]); sY = atof(argv[5]);
        cmdVelTopic = argv[6];
        laserTopic = argv[7];
        maxLinearVel = atof(argv[8]);
        maxAngularVel = atof(argv[9]);
        linearAcc = atof(argv[10]);
        angularAcc = atof(argv[11]);
        goalDistDiff = atof(argv[12]);
        goalAngleDiff = atof(argv[13]);
        planner_type = argv[14];
        angularResolution = atof(argv[15]);
        maxObstacleDistance = atof(argv[16]);
        footprintTopic = argv[17];
        planTopic = argv[18];
    }else{
        cout<<argc<<endl;
        ROS_INFO("PARAMETROS INCORRECTOS");
        return 0;
    }

    cout<<"**********************************************************************"<<endl;
    cout<<"PARÁMETROS DEL NAVEGADOR"<<endl;
    cout<<"camino:   "<<pathTopic<<endl;
    cout<<"robot f:  "<<robotFrame<<endl;
    cout<<"robot p:  "<<poseTopic<<endl;
    cout<<"tamaño:   "<<sX<<" x "<<sY<<endl;
    cout<<"cmd_vel:  "<<cmdVelTopic<<endl;
    cout<<"laser:    "<<laserTopic<<endl;

    cout<<"max vel:  "<<maxLinearVel<<endl;
    cout<<"max ang:  "<<maxAngularVel<<endl;
    cout<<"lin acc:  "<<linearAcc<<endl;
    cout<<"ang acc:  "<<angularAcc<<endl;
    cout<<"dis2goal: "<<goalDistDiff<<endl;
    cout<<"ang2goal: "<<goalAngleDiff<<endl;

    cout<<"planner:  "<<planner_type<<endl;
    cout<<"angRes:   "<<angularResolution<<endl;
    cout<<"dis2obst: "<<maxObstacleDistance<<endl;

    cout<<"ftprint:  "<<footprintTopic<<endl;
    cout<<"planPath: "<<planTopic<<endl;

    cout<<"**********************************************************************"<<endl;

    // Inicializar PathFollower
    //PathFollower(string robotPoseTopic, double sX, double sY, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double posDiff, double angleDiff, string planner_type, double angularResolution, double maxObstacleDistance, string footprintTopic, string planTopic); // Constructor para seguimiento simple
    pf = new PathFollower(robotFrame, poseTopic, sX, sY, cmdVelTopic, laserTopic, maxLinearVel, maxAngularVel, linearAcc, angularAcc, goalDistDiff, goalAngleDiff, planner_type, angularResolution, maxObstacleDistance, footprintTopic, planTopic);

    // Suscribir al path follower
    ros::Subscriber pfSub = nh.subscribe<nav_msgs::Path>(pathTopic, 1, &pathPosesCallback);
    //pf.followPath(path2GoalPoses);


    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;

    ros::spin();

    return 0;

}