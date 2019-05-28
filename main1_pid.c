#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#include <motor_turn.h>
#include <zumo_config.h>
#include <inttypes.h>
#include <stdint.h>

 #if 0
// PID controller
float Kp=120;
float Ki=0;
float Kd=10;

int error=0, P=0, D=0, PIDvalue=0;
int previousError=0;
float differ;
int max_speed = 255;
int min_speed = 0;
struct sensors_ dig;

void init() {
    motor_start();              // enable motor controller
    IR_Start();
    IR_flush(); // clear IR r
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 20000, 20000, 9000, 9000);
}

void get_error() {
    reflectance_digital(&dig);
    if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 1) 
        error = -5;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 1 && dig.r3 == 1)
        error = -4;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 1 && dig.r3 == 0)
        error = -3;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 0)
        error = -2;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0)
        error = -1;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0)
        error = 0;
    else if (dig.l3 == 1 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) 
        error = 5;
    else if (dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0)
        error = 4;
    else if (dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0)
        error = 3;
    else if (dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0)
        error = 2;
    else if (dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0)
        error = 1;
   
}

void calculate_pid() {
    get_error();
    P = error;
    D = error-previousError;
    PIDvalue = (Kp*P) + (Kd*D);
    previousError = error;
}

void motor_move() {
    
    int left_speed  = max_speed - PIDvalue; 
    int right_speed = max_speed + PIDvalue; 
    
    
    if (left_speed > max_speed)
        left_speed = max_speed;
    
    if (right_speed > max_speed)
        right_speed = max_speed;
    
    if (left_speed < min_speed)
        left_speed = min_speed;
    
    if (right_speed < min_speed)
        right_speed = min_speed;
    
    motor_turn(left_speed, right_speed, 5);
}

//motor v 6 high battery
void zmain(void)
{
    struct sensors_ ref;
    int counter = 0;
    
    int sum = 0, sum_ir = 0;;
    reflectance_start();
    bool start_while= true;
    bool main_while,line_count,xx=false;
    TickType_t start;
    TickType_t end;

    // initialization
    init();
    
    // First line loop
    while(1){
        if(SW1_Read() == 0) {
            vTaskDelay(1000);
            sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;
            
            while(sum != 6) {
                motor_forward(50,50);  
                reflectance_digital(&dig); 
                sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;
           
                if(sum == 6){
                    vTaskDelay(300); 
                    motor_forward(0,0);
                    print_mqtt("Zumo038/ready ","line");
                    IR_wait();
                    counter++;
                    start = xTaskGetTickCount();
                    print_mqtt("Zumo038/start ","%.2d ",start);
                    motor_forward(255,300);
                }
            }
            break;
        }
    }
 
    // Second loop
    while(1) {
        reflectance_digital(&dig);

        if(dig.l3==1 && dig.r3==1){
            if(dig.l3==1 && dig.r3==1 && counter==1){
                motor_forward(255,50);
                counter++;
            }

            reflectance_digital(&dig);
            if(dig.l3==1 && dig.r3==1 && counter==2){
                end = xTaskGetTickCount(); 
                differ=(float)(end-start)/1000; 
                print_mqtt("Zumo038/stop ","%.2d ",end);
                print_mqtt("Zumo038/time ","%.2f seconds",differ);
                counter++;
                motor_forward(0,0);
                motor_stop();
                vTaskDelay(3000);
                break;
            }
        }
     
        else {
            calculate_pid();
            motor_move();
        }
    }     
 } 
   
#endif
