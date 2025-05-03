#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>

#define TIME_STEP 16
#define NUM_SENSORS 9  // Jumlah sensor IR
#define MAX_SPEED 17 // Kecepatan maksimum motor
#define MAX_DIR 100

const char *sensor_names[NUM_SENSORS] = {"ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7", "ir8"};
WbDeviceTag sensors[NUM_SENSORS];



double constrain(double nilai, double min, double max) {
    if (nilai < min) {
        return min;
    } else if (nilai > max) {
        return max;
    } else {
        return nilai;
    }
}


int main() {
    wb_robot_init();

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i] = wb_robot_get_device(sensor_names[i]);
        wb_distance_sensor_enable(sensors[i], TIME_STEP);
    }
    WbDeviceTag left_motor = wb_robot_get_device("left motor");
    WbDeviceTag right_motor = wb_robot_get_device("right motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
     double Kp = 3;
     double Ki = 0.0;
     double Kd = 2.8;
  
     double error = 0;
     double last_error = 0;
     double integral = 0;
     double forward = 12.0;
    
    char dir = 'F';
    char prev_dir = 'F' ;
    char get_dir = 'A';
    char prev_get_dir = 'A';
    bool change = true;
    bool c_for = true;
    double weighted_sum = 0.0;
    char direction[MAX_DIR] = {'S', 'L'};  // Array untuk menyimpan arah
    int dir_index = 0;        // Indeks arah terakhir
    //-----------------------------------------------------------------
    
void belok_kanan(){
   wb_motor_set_velocity(left_motor,  0.8 * MAX_SPEED);
   wb_motor_set_velocity(right_motor, 0.1 * -MAX_SPEED);
   }
void muter(){
 wb_motor_set_velocity(left_motor, 0.35 * -MAX_SPEED);
 wb_motor_set_velocity(right_motor,0.35 * MAX_SPEED);
 }
void belok_kiri(){
 wb_motor_set_velocity(left_motor, 0.1 * -MAX_SPEED);
  wb_motor_set_velocity(right_motor,0.75 * MAX_SPEED);
 }
void stop(){
wb_motor_set_velocity(left_motor, 0 * MAX_SPEED);
  wb_motor_set_velocity(right_motor,0 * -MAX_SPEED);
}

void jalan_lurus(){     
    double position = 0.0;
    double sum = 0.0;

    for (int i = 0; i < 7; i++) {
      double value = wb_distance_sensor_get_value(sensors[i]);
      double weight = i - 3;  // posisi relatif dari -4 sampai +4
      position += weight * value;
      sum += value;
    }
    if (sum != 0)
      position /= sum;   
    error = position;
    integral += error;
    double derivative = error - last_error;
    double correction = Kp * error + Ki * integral + Kd * derivative;
    weighted_sum = sum;
    last_error = error;
    double left_speed = forward - correction;
    double right_speed = forward + correction;
    left_speed =  constrain(left_speed, -15.0, 15.0);
    right_speed = constrain(right_speed, -15.0, 15.0);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
}   


    while (wb_robot_step(TIME_STEP) != -1) {
        double sensor_values[NUM_SENSORS];     
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
        }
         // printf("KANAN-Sensor 0: %.2f, Sensor 1: %.2f, Sensor 2: %.2f, Sensor 3: %.2f, Sensor 4: %.2f, Sensor 5: %.2f, Sensor 6: %.2f -KIRI\n", sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4], sensor_values[5], sensor_values[6]);
         // printf("KABA-Sensor 7: %.2f, Sensor 8: %.2f -KIBA \n", sensor_values[7], sensor_values[8]);
         printf("direction: %c \t, forward: %.2f \t, change: %d \t", dir, weighted_sum, change);
         // printf("direction: %c , forward: %.2f \n", dir, forward);
        
        //Telusur Kiri dan Kanan Pasti Ada
         if (sensor_values[4] > 500 && sensor_values[8] > 500 && sensor_values[7] > 500 && sensor_values[0] < 300 && sensor_values[6] < 300 && change == true){                   
           printf("Perempatan \n");
           dir = direction[dir_index++];
           change = false;
           }   
         else if (sensor_values[8] < 300 && sensor_values[7] < 300 && sensor_values[3] < 300 && weighted_sum < 998 && change == true){       
           dir = direction[dir_index++];     
           printf("U-Turn\n");
           change  = false;
           }   
         else if (sensor_values[0] > 500 && sensor_values[1] > 500 && sensor_values[2] > 500 && sensor_values[3] > 500 && sensor_values[4] > 500 && sensor_values[5] > 500 && sensor_values[0] > 500 && sensor_values[7] > 500 && sensor_values[8] > 500){       
           forward = 0.0;
           dir = 'B';
           printf("Target Found!!!! \n");       
           }   
           
         // //Telusur Kanan dak Kiri tergantung Opsi  
         // else if (sensor_values[3] < 200  &&  sensor_values[7] > 500 && sensor_values[8] > 500 && change == true ){       
           // printf("Simpang T \n");
           // change = false;
           // }
           
         // //Kondisi Telusur saling membutuhkan
         // else if (sensor_values[7] > 500 && sensor_values[8] < 200 && sensor_values[3] > 500 && change == true){       
           // printf("Simpang Tiga-Kanan \n");
           // if(Mode_Telusur == true){change = false;}
           // }
           
         else if (sensor_values[8] > 500 && sensor_values[7] < 200 && sensor_values[3] > 500 && change == true){            
           dir = direction[dir_index++];  
           printf("Simpang Tiga-Kiri  \n");
           change = false;
           }
           
          else if (sensor_values[3] < 300 &&  sensor_values[7] < 300 && sensor_values[8] > 500 && change == true){       
           dir = 'l';
           printf("Belok kiri ikut jalan  \n");
           change = false;
           }
          
          //Berhenti sebentar ketika berganti arah gerak Robot
              if (dir != prev_dir){
                int delay = 10;
                printf("Stop 100ms \n"); 
                if (dir=='S'){delay = 5;}
                else{delay=100;}
                c_for = false;
                stop();
                int steps = delay / TIME_STEP;
                for (int i = 0; i < steps; i++) {
                    wb_robot_step(TIME_STEP); 
                    }      
                prev_dir = dir;
                }
                printf("cek \n");
              
               if(sensor_values[8] < 300 && sensor_values[7] < 300 && dir == 'S'){
                dir = 'F';
                change = true;
                }
               else{
                  if(sensor_values[3] < 200){
                    c_for = true;
                    printf("Switch to True \n");            
                    }   
                  else if(sensor_values[3] > 500 && c_for == true){
                    dir = 'F';
                    get_dir = 'F';
                    change = true;
                    printf("Jalan Maju \n");   
                    }}
               
         switch (dir) {
                    case 'F':
                      jalan_lurus();
                      break;
                    case 'R':
                      belok_kanan();       
                      break;
                    case 'L':
                      belok_kiri();       
                      break;
                    case 'l':
                      belok_kiri();             
                      break;
                    case 'U':
                       muter();              
                      break;
                    case 'B':
                       stop();              
                      break;
                    case 'S':
                       jalan_lurus();              
                      break;

                  }
                   
}
    wb_robot_cleanup();
    return 0;
}
