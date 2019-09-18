#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"									              			  	// Inclusao da bilioteca responsavel pelo comando que fornecera a posicao desejada
#include "turtlesim/Pose.h"											                			// Inclusao da bilioteca responsavel pelo topico "/turtle1/pose", que fornece a posicao atual da tartaruga
#include "geometry_msgs/Twist.h"										            		    // Inclusao da bilioteca responsavel pelo topico "/turtle1/cmd_vel", que fornece como a tartaruga sera deslocada

// Declaracao das funcoes utilizadas no main
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& Info);
void CurPoseCallback(const turtlesim::Pose::ConstPtr& Info);
float Linear_Error(turtlesim::Pose Current_Position, geometry_msgs::Pose2D Desired_Position);
float Angular_Error(turtlesim::Pose Current_Position, geometry_msgs::Pose2D Desired_Position);

// Declaracao das variaveis globais
bool Loop_Variable = true;
turtlesim::Pose Current_Position;
geometry_msgs::Pose2D Desired_Position;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Go_To_Goal_Turtlesim_pubsub");      	        	          		    // Comando necessario para conectar com o roscore
	ros::NodeHandle n;													             		 	// Inicializacao do node
	
	ros::Subscriber Desired_Position_sub = n.subscribe("/turtle1/PositionCommand", 5, ComPoseCallback);
	ros::Subscriber Current_Position_sub = n.subscribe("/turtle1/pose", 5, CurPoseCallback);
	ros::Publisher Velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
	
	// Declaracao de variaveis
	ros::Rate loop_rate(10);
	float Lin_Error = 0;
	float Ang_Error = 0;
	geometry_msgs::Twist Velocity_Value;
	
	ROS_INFO("Pronto para receber comandos");
	while (ros::ok() && n.ok() )
	{
		ros::spinOnce();
		if (Loop_Variable == false)
		{
			
			Lin_Error = Linear_Error(Current_Position, Desired_Position);						// Erro linear encontrado na funcao "Linear_Error"
			Ang_Error = Angular_Error(Current_Position, Desired_Position);						// Erro angular encontrado na funcao "Angular_Error"
			printf("Erro linear: %f, Erro angular: %f\n", Lin_Error, Ang_Error);
			
			//Implementacao do controle de acordo com o erro
			if (Lin_Error > 0.1)
			{
				Velocity_Value.linear.x = 0.2 * Lin_Error;
			}
			else
			{
				Velocity_Value.linear.x = 0;
			}
			
			Velocity_Value.angular.z = 0.5 * Ang_Error;
			Velocity_pub.publish(Velocity_Value);								                // Publica para deslocar a tartaruga
		}
		else
		{
			printf("Aguardando comandos...\n");
		}
		loop_rate.sleep();
	}
}


// Funcao responsavel por enviar a nova posicao desejada
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& Info)			
{
	Loop_Variable = false;													         		    // Inicializacao do while dentro no main
	Desired_Position.x = Info->x;
	Desired_Position.y = Info->y;
	return;
}

// Funcao responsavel por enviar a nova posicao atual
void CurPoseCallback(const turtlesim::Pose::ConstPtr& Info)			
{
	Current_Position.x = Info->x;
	Current_Position.y = Info->y;
	Current_Position.theta = Info->theta;
	return;
}

// Funcao responsavel por fornecer o erro angular do deslocamento da tartaruga
float Angular_Error(turtlesim::Pose Current_Position, geometry_msgs::Pose2D Desired_Position)
{
	float Ex = Desired_Position.x - Current_Position.x;											// Erro na componente X
	float Ey = Desired_Position.y - Current_Position.y;											// Erro na componente Y
	float dest = atan2f(Ey, Ex); 										              		    // Obtencao do angulo entre os vetores dos erros
	float Et = dest - Current_Position.theta;                                         		    // Obtencao do erro angular
	return Et;
}

// Funcao responsavel por fornecer o erro linear do deslocamento da tartaruga
float Linear_Error(turtlesim::Pose Current_Position, geometry_msgs::Pose2D Desired_Position)
{
	float Ex = Desired_Position.x - Current_Position.x;											// Erro na componente X
	float Ey = Desired_Position.y - Current_Position.y;											// Erro na componente Y
	float Et = Angular_Error(Current_Position, Desired_Position);								// Obtencao do angulo entre os vetores dos erros
	float Etx = hypotf(Ex, Ey)*cos(Et);                                             		    // Projecao do erro no eixo x da tartaruga
	return Etx;
}