/*
 * File:          Controlador2.c
 * Date:          12/09/2021
 * Description:
 * Author:        Andy Barbosa
 * Modifications:
 */

#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>

// Conjunto de funções disponíveis para cada robô.
#include <webots/supervisor.h>

#define MAX_SPEED 6.28

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796

#define RAD_THRESHOLD 0.035

int TIME_STEP;
WbDeviceTag left_motor, right_motor;

void get_min_distance(double x_robot, double z_robot, double boxes[9][3], int *target, double *tangent){
  int i = 0;
  double min_distance = 1000;
  double temp = 0.0;
  int caixa_alvo = 0;
  double tangent_nearest = 0.0;
  for (i = 0; i < 9; i++){
    // Caixa não foi visitada, pode ser considera na descoberta de menor distância.
    if(boxes[i][2] == 0){
      // Fórmula de distância entre dois pontos.
      temp = sqrt( pow((boxes[i][0] - x_robot),2) + pow((boxes[i][1] - z_robot),2) );
      //printf("Caixa %d | Tangente: %6.2f\n",i,tan( ((x_robot - boxes[i][0])/(z_robot-boxes[i][1])) ));
      if (temp <= min_distance){
        min_distance = temp;
        // Salva a caixa que possui a menor distância.
        caixa_alvo = i;
        // Tangente = DeltaX/DeltaZ
        tangent_nearest = tan( (boxes[i][0] - x_robot)/(boxes[i][1] - z_robot) );
      }
    } 
  }
  *target = caixa_alvo;
  *tangent = tangent_nearest;
}

void stop_motor(){
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0); 
}

void move_forward(){
  wb_motor_set_velocity(left_motor, 0.25*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.25*MAX_SPEED); 
}

void rotate_left(){
  wb_motor_set_velocity(left_motor, -0.25*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.25*MAX_SPEED); 
}

void rotate_right(){
  wb_motor_set_velocity(left_motor, 0.25*MAX_SPEED);
  wb_motor_set_velocity(right_motor, -0.25*MAX_SPEED);  
}

void sync_move(int move_type) {
  float heading;
  if(move_type == 1){
    rotate_left();
    heading = 60;
  }
  else{
    move_forward();
    heading = 200;
  }
  
  float duration = heading / (ROBOT_ANGULAR_SPEED_IN_DEGREES*0.25);
  float start_time = wb_robot_get_time();
  do
  {
   wb_robot_step(TIME_STEP);
  } while (wb_robot_get_time() < start_time + duration);

}



// Main code
int main(int argc, char **argv) {
  // inicializa Webots API
  wb_robot_init();
  
  TIME_STEP = (int)wb_robot_get_basic_time_step();

  double left_speed = 0;  //Velocidade das roda esquerda
  double right_speed = 0; //Velocidade das roda direita
  int i; // Variavel para o for.
  bool adjust_angle = false; // Controle de ângulo do robô
  int orientacao = 0;
  
  int target = -1; // Caixa alvo
  double tangent; // Tangente entre robo e caixa da caixa mais próxima
  double correct_angle = 0; // ângulo que o robô deve seguir para bater na caixa
  double correct_rotation; // ângulo arredondado para evitar imprecisões
  
  // Sensor de proximidade do robô
  WbDeviceTag ps[8];
  
  // Nome de cada sensor, perceba que existem 8 strings
  // com 3 caracteres cada, por isso a dimensão [8][4].
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  // Mapa de caixas dispostas no cenário (X,Z,status)
  double boxes[9][3] = {
    {-0.25,0.25, 0},  
    {-0.25,0, 0},     
    {-0.25,-0.25, 0}, 
    {0, 0.25, 0},     
    {0, 0, 0},        
    {0,-0.25, 0},     
    {0.25, 0.25, 0}, 
    {0.25,0, 0},      
    {0.25,-0.25, 0}   )
  };
  
  for (i = 0; i < 8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("e-Puck");
  
  WbFieldRef trans_field = wb_supervisor_node_get_proto_field(robot_node, "translation");
  
  WbFieldRef rotac_field = wb_supervisor_node_get_proto_field(robot_node, "rotation");
  
  const double *posicao_robo; // Salva a posição do robô (posteriormente)
  const double *rotacao_robo; // Salva a rotação do robô (posteriormente)
  

  
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
  
    // Leitura da saída dos sensores
    double ps_values[8];
    for (i = 0; i < 8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    // Detecta obstáculos
    bool right_obstacle = ps_values[0] > 140.0 || ps_values[1] > 140.0;
    bool left_obstacle = ps_values[6] > 140.0 || ps_values[7] > 140.0;

    if(correct_rotation - RAD_THRESHOLD > 1.57 
    && correct_rotation - RAD_THRESHOLD < M_PI){
      orientacao = 1;
    }

    if (left_obstacle || right_obstacle) {
      boxes[target][2] = 1;
      printf("Vou virar 180\n");
      sync_move(1);
      sync_move(0);
      target = -1;
      orientacao = 0;
    }
    
    
    // Atualiza o vetor de posições do robô ([0]: X, [1]: Y, [2]: Z)
    posicao_robo = wb_supervisor_field_get_sf_vec3f(trans_field);
    
    // Robô não tem uma caixa alvo
    if (target == -1){
      get_min_distance(posicao_robo[0],posicao_robo[2],boxes, &target, &tangent);
      printf("CAIXA QUE DEVE IR: %d | TANGENTE: %6.2f | ARCTAN: %6.2f \n",target, tangent, atan(tangent));
      stop_motor();
    
    }
    // Cálculo do ângulo do robô em direção à caixa mais próxima
    if (posicao_robo[0] > boxes[target][0] && posicao_robo[2] > boxes[target][1])
      correct_angle = atan(tangent);
      
    
    else if (posicao_robo[2] < boxes[target][1] && posicao_robo[0] > boxes[target][0])
      correct_angle = M_PI - fabs(atan(tangent));
    
    
    else if (posicao_robo[0] < boxes[target][0] && posicao_robo[2] < boxes[target][1])
      correct_angle = atan(tangent) - M_PI;
      
    
    else if (posicao_robo[2] > boxes[target][1] && posicao_robo[0] < boxes[target][0])
      correct_angle = atan(tangent);     
        
    // Atualiza o vetor de rotação do robô ([0]: X, [1]: Y, [2]: Z, [3]: ângulo)
    rotacao_robo = wb_supervisor_field_get_sf_rotation(rotac_field);
    correct_rotation = (round(rotacao_robo[3] * 1000.0) / 1000.0);
    correct_angle = round(correct_angle * 1000.0) / 1000.0;
    
    // Inversão dos radianos para condizer as reais posições.    
    if (correct_rotation >= 0.00){
      if (correct_angle < 0.00)
        correct_angle = -1*correct_angle;
    }
    else{
      if (correct_angle > 0.00)
        correct_angle = -1*correct_angle;
    }
        
    printf("Posição Robo: %6.2f  %6.2f\n",posicao_robo[0], posicao_robo[2]);
    printf("Rotação: %f\n",correct_rotation);
    printf("ÂNGULO PRA CONSIDERAR: %f\n", correct_angle);
        
    if (correct_rotation < correct_angle - RAD_THRESHOLD
    || correct_rotation > correct_angle + RAD_THRESHOLD){
      printf("Gira\n");
      if(boxes[target][1] > posicao_robo[2]){
        if (boxes[target][0] > posicao_robo[0]){
          rotate_right();
        }
        else if (boxes[target][0] < posicao_robo[0]){
          rotate_left();
        }
      }
      else if(boxes[target][1] < posicao_robo[2]){
        if (boxes[target][0] > posicao_robo[0]){
          rotate_right();
        }
        else if (boxes[target][0] < posicao_robo[0]){
          if(orientacao == 1){
            rotate_right();  
          }
          else{
            rotate_left();
          }
        }
      }
    }
    else{
      move_forward();
    }
  }

  wb_robot_cleanup();
  return 0;
}