#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>

#define TIME_STEP 8
#define NUM_SENSORS 9  // Jumlah sensor IR
#define MAX_SPEED 10 // Kecepatan maksimum motor
#define MAX_DIR 100



int main() {
    wb_robot_init();
    WbDeviceTag ir_sensors[NUM_SENSORS];
    char sensor_names[NUM_SENSORS][10] = {"ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7", "ir8"};
    for (int i = 0; i < NUM_SENSORS; i++) {
        ir_sensors[i] = wb_robot_get_device(sensor_names[i]);
        wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
    }
    WbDeviceTag left_motor = wb_robot_get_device("left motor");
    WbDeviceTag right_motor = wb_robot_get_device("right motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    double weights[NUM_SENSORS] = {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
    double Kp = 0.001;  
     // double sensor_values[NUM_SENSORS];
    char dir = 'F';
    char prev_dir = 'F' ;
    char get_dir = 'C';
    bool change = true;
    bool rotate = true;
    double forward = 1.0;
    double weighted_sum = 0.0;
    char direction[MAX_DIR];  // Array untuk menyimpan arah
    int dir_index = 0;        // Indeks arah terakhir
    
 

void telusur_kanan(){
}


void belok_kanan(){
 wb_motor_set_velocity(left_motor, forward * 0.75 * MAX_SPEED);
 wb_motor_set_velocity(right_motor,forward * 0.1 * -MAX_SPEED);
 

}
void muter(){
 wb_motor_set_velocity(left_motor, 0.3 * -MAX_SPEED);
 wb_motor_set_velocity(right_motor,0.3 * MAX_SPEED);
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
        double sensor_values[NUM_SENSORS];
        weighted_sum = 0.0;
        // Membaca nilai sensor dan menghitung nilai berbobot
        for (int i = 0; i < 7; i++) {
            sensor_values[i] = wb_distance_sensor_get_value(ir_sensors[i]);
            weighted_sum += sensor_values[i] * weights[i];
        }

        // Menentukan error (target adalah 0)
        double error = weighted_sum;
        
        // Menghitung koreksi kecepatan
        double correction = Kp * error;

        // Menyesuaikan kecepatan roda
        double left_speed = MAX_SPEED - correction;
        double right_speed = MAX_SPEED + correction;

        // Membatasi kecepatan dalam rentang [-MAX_SPEED, MAX_SPEED]
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
        if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
        if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

        // Mengatur kecepatan motor
        wb_motor_set_velocity(left_motor, forward * left_speed);
        wb_motor_set_velocity(right_motor,forward * right_speed);
}   


    // Loop utama
    while (wb_robot_step(TIME_STEP) != -1) {
        double sensor_values[NUM_SENSORS];     
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensor_values[i] = wb_distance_sensor_get_value(ir_sensors[i]);
        }
         // printf("KANAN-Sensor 0: %.2f, Sensor 1: %.2f, Sensor 2: %.2f, Sensor 3: %.2f, Sensor 4: %.2f, Sensor 5: %.2f, Sensor 6: %.2f -KIRI\n", sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4], sensor_values[5], sensor_values[6]);
         // printf("KABA-Sensor 7: %.2f, Sensor 8: %.2f -KIBA \n", sensor_values[7], sensor_values[8]);
         printf("direction: %c , weight: %.2f  change: %d \n", dir, weighted_sum, change);
         // printf("direction: %c , forward: %.2f \n", dir, forward);
               
        //prapatan
         if (sensor_values[4] > 500 && sensor_values[8] > 500 && sensor_values[7] > 500 && sensor_values[0] < 300 && sensor_values[6] < 300){                   
           dir = 'R';
           get_dir = 'R';
           printf("Perempatan \n");
           }          
         else if (sensor_values[3] < 200  &&  sensor_values[7] > 500 && sensor_values[8] > 500 && change == true ){       
           dir = 'R';
           get_dir = 'R';
           printf("D2 \n");
           }
         else if (sensor_values[7] > 500 && sensor_values[8] < 200 && sensor_values[3] > 500){       
           dir = 'R';
           get_dir = 'R';
           printf("Pertigaan \n");
           }
         else if (sensor_values[8] < 300 && sensor_values[7] < 300 && sensor_values[3] < 300 && weighted_sum < -0.19 && weighted_sum > -0.36){       
           dir = 'U';
           get_dir = 'U';
           change  = true;
           printf("U-Turn\n");
           }
          else if (sensor_values[3] < 300 &&  sensor_values[7] < 300 && sensor_values[8] > 500 && weighted_sum < -0.40 && weighted_sum > -0.50 && change == true){       
           dir = 'L';
           get_dir = 'N';
           printf("Left-Road  \n");
           }
          else if (sensor_values[0] > 500 && sensor_values[1] > 500 && sensor_values[2] > 500 && sensor_values[3] > 500 && sensor_values[4] > 500 && sensor_values[5] > 500 && sensor_values[0] > 500 && sensor_values[7] > 500 && sensor_values[8] > 500){       
           forward = 0.0;
           dir = 'B';
           get_dir = 'N';
           printf("Target Found!!!! \n"); 
           for (int i = 0; i < dir_index; i++) {
                printf("%c ", direction[i]);
            }
            printf("\n");        
           }
         
         
         else if(sensor_values[3] < 200){
         change = true;
          }
         else if(sensor_values[3] > 500 && change == true){
          get_dir = 'N';
          dir = 'F';
         }
         
          if (dir != prev_dir){
              change = false;
              int delay = 100; 
              if (get_dir != 'N') {
              direction[dir_index++] = get_dir;
              }
              printf("STOP!!! \n");
              stop();
              int steps = delay / TIME_STEP;
              for (int i = 0; i < steps; i++) {
                  wb_robot_step(TIME_STEP); 
              }        
              prev_dir = dir;
              }
              
          
          
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
                       printf("cek");             
                      break;
                    case 'U':
                       muter();              
                      break;
                    case 'B':
                       stop();              
                      break;
                  }
               
                   
}
    wb_robot_cleanup();
    return 0;
}
