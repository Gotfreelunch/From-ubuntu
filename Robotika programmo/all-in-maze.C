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
WbDeviceTag left_motor;
WbDeviceTag right_motor;

double Kp = 3;
double Ki = 0.0;
double Kd = 2.8;

double error = 0;
double last_error = 0;
double integral = 0;
double forward = 12.0;

double sensor_values[NUM_SENSORS];
char dir = 'F';
char prev_dir = 'F' ;
char get_dir = 'A';
char prev_get_dir = 'A';
bool change = true;
bool c_for = true;
double weighted_sum = 0.0;
char direction[MAX_DIR];  // Array untuk menyimpan arah
int dir_index = 0;        // Indeks arah terakhir
char short_dir[50];
int idxSecond = 0;
//-----------------------------------------------------------------
bool Mode_Telusur = true; //Ganti ke false untuk mode telusur kiri
bool Mode_Robot = true; //Ganti ke false untuk mode short path
//-----------------------------------------------------------------
//Variabel untuk Mode Telusur Kanan dan Kiri
char perempatan_dir = 'O';
char simpang_T = 'O';
char stKA = 'O';
char stKI = 'O';
char DstKA = 'O';
char DstKI = 'O';
char finder_choose = 'O';


double constrain(double nilai, double min, double max) {
    if (nilai < min) {
        return min;
    } else if (nilai > max) {
        return max;
    } else {
        return nilai;
    }
}

void shortpath(char input[], int inputLength, char choose) {
  char counter_choose = 'O';
  char firstPass[50];
  int idxFirst = 0;
  int i = 0;
  if(choose == 'R'){counter_choose = 'L';}
  else{counter_choose = 'R';}
  
  while (i < inputLength) {
    if (i + 2 < inputLength && input[i] == choose && input[i+1] == 'U' && input[i+2] == choose) {
      firstPass[idxFirst++] = 'S';
      i += 3;
    } else {
      firstPass[idxFirst++] = input[i];
      i++;
    }
  }

  char secondPass[50];
  int idxSecond = 0;
  int j = 0;

  while (j < idxFirst) {
    if (j + 2 < idxFirst &&
        ((firstPass[j] == 'S' && firstPass[j+1] == 'U' && firstPass[j+2] == choose) ||
         (firstPass[j] == choose && firstPass[j+1] == 'U' && firstPass[j+2] == 'S'))) {
      secondPass[idxSecond++] = counter_choose;
      j += 3;
    } else {
      secondPass[idxSecond++] = firstPass[j];
      j++;
    }
  }

  printf("Short Path : ");
  for (int k = 0; k < idxSecond; k++) {
    printf("%c ", secondPass[k]);
  }
  printf("\n");
}

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
      double value = sensor_values[i];
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

void searchtracklogic(){
   //Ambil dan Rekam perjalanan
   if(get_dir != prev_get_dir){
      if (get_dir != 'F') {
             direction[dir_index++] = get_dir;}
      prev_get_dir = get_dir;}
   //Berhenti sebentar ketika berganti arah gerak Robot
   if (dir != prev_dir){
        printf("Stop 100ms \n"); 
          int delay = 100;
          c_for = false;
          stop();
          int steps = delay / TIME_STEP;
          for (int i = 0; i < steps; i++) {
              wb_robot_step(TIME_STEP); 
          } prev_dir = dir;}  
   else if(sensor_values[3] < 200){
      c_for = true;}   
   else if(sensor_values[3] > 500 && c_for == true){
      dir = 'F';
      get_dir = 'F';
      change = true;}
}

void shortpathlogic(){
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
}


void Main_Logic(bool Mode){
     if (sensor_values[4] > 500 && sensor_values[8] > 500 && sensor_values[7] > 500 && sensor_values[0] < 300 && sensor_values[6] < 300 && change == true){                   
       printf("Perempatan \n");
       if(Mode == true){
         dir = perempatan_dir;
         get_dir = dir;
         change = false;}
        else{ 
         dir = direction[dir_index++];
         change = false;}
       }   
     else if (sensor_values[8] < 300 && sensor_values[7] < 300 && sensor_values[3] < 300 && weighted_sum < 999 && change == true){       
       printf("U-Turn\n");
       if(Mode == true){
         dir = 'U'; 
         get_dir = dir;      
         change  = false;}
       else{
         dir = direction[dir_index++];     
         change  = false;}
       }   
     else if (sensor_values[0] > 500 && sensor_values[1] > 500 && sensor_values[2] > 500 && sensor_values[3] > 500 && sensor_values[4] > 500 && sensor_values[5] > 500 && sensor_values[0] > 500 && sensor_values[7] > 500 && sensor_values[8] > 500 && change == true){       
       printf("Target Found!!!! \n");
       if(Mode == true){
         dir = 'B';
         printf("Tracked Direction: ");
         for (int i = 0; i < dir_index; i++) {
              printf("%c ", direction[i]);
          }
         printf(" \n");
         shortpath(direction, dir_index, finder_choose);
         change = false;}
        else{
         dir = 'B';}
       }  
        
     else if (sensor_values[3] < 200  &&  sensor_values[7] > 500 && sensor_values[8] > 500 && change == true ){       
       printf("Simpang T \n");
       if(Mode == true){
         dir = simpang_T;
         get_dir = dir;
         change = false;}
       else{
         dir = direction[dir_index++];  
         change = false;}
       }
       
     //Kondisi Telusur saling membutuhkan
     else if (sensor_values[7] > 500 && sensor_values[8] < 200 && sensor_values[3] > 500 && change == true){       
       printf("Simpang Tiga-Kanan \n");
       if(Mode == true){
         dir = stKA;
         get_dir = DstKA;
         if(Mode_Telusur == true){change = false;}}
       else{
         dir = direction[dir_index++]; 
         change = false;}
       }
       
     else if (sensor_values[8] > 500 && sensor_values[7] < 200 && sensor_values[3] > 500 && change == true){            
       printf("Simpang Tiga-Kiri  \n");
       if(Mode == true){
         dir = stKI;
         get_dir = DstKI; 
         if(Mode_Telusur == false){change = false;}}
       else{
         dir = direction[dir_index++];  
         change = false;}
       }
       
      else if (sensor_values[3] < 200 &&  sensor_values[7] < 200 && sensor_values[8] > 500 && change == true){       
       dir = 'l';
       printf("Belok kiri ikut jalan  \n");
       change = false;
       }
      else if (sensor_values[3] < 300 &&  sensor_values[8] < 300 && sensor_values[7] > 500 && change == true){       
       dir = 'r';
       printf("Belok kanan ikut jalan  \n");
       change = false;
       }
       
      //Logic untuk percabangan  
      if(Mode == true){
      searchtracklogic();
      }else{shortpathlogic();}   
}



int main() {//Void Setup
    wb_robot_init();
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i] = wb_robot_get_device(sensor_names[i]);
        wb_distance_sensor_enable(sensors[i], TIME_STEP);
    }
    left_motor = wb_robot_get_device("left motor");
    right_motor = wb_robot_get_device("right motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
   if(Mode_Telusur == true){
    perempatan_dir = 'R';
    simpang_T = 'R';
    stKA = 'R';
    stKI = 'F';
    DstKA = 'R';
    DstKI = 'S';
    finder_choose = 'R';
    printf("Mode Telusur Kanan dimulai!! \n");
   }
   else{
    perempatan_dir = 'L';
    simpang_T = 'L';
    stKA = 'F';
    stKI = 'L';
    DstKI = 'L';
    DstKA = 'S';
    finder_choose = 'L';
    printf("Mode Telusur Kiri dimulai!! \n");
   }


while (wb_robot_step(TIME_STEP) != -1) { //Main Loop
             
        for (int i = 0; i < NUM_SENSORS; i++) { //Get sensor
            sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
        }      
        Main_Logic(Mode_Robot);
        
         switch (dir) {
                    case 'F':
                      jalan_lurus();
                      break;
                    case 'R':
                      belok_kanan();       
                      break;
                     case 'r':
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
                  }
                   
}
    wb_robot_cleanup();
    return 0;
}
