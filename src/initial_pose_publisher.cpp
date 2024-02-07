#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "initial_pose_publisher");
  ros::NodeHandle nh;

  ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

  // Crear el mensaje de posición inicial
  geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
  initialPoseMsg.header.frame_id = "map";
  initialPoseMsg.pose.pose.position.x = 1.0;  // Coordenada x deseada
  initialPoseMsg.pose.pose.position.y = 2.0;  // Coordenada y deseada
  initialPoseMsg.pose.pose.orientation.w = 1.0;  // Orientación deseada (cuaternión)

  // Publicar la posición inicial
  initialPosePub.publish(initialPoseMsg);

  // Esperar un breve periodo de tiempo para asegurarse de que se publique el mensaje
  ros::Duration(0.5).sleep();

  // Continuar con el resto de tu código...


  

  return 0;
}