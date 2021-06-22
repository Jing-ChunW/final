#include"mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"
#include "mbed_rpc.h"
#include <math.h>

BufferedSerial pc(USBTX,USBRX); //tx,rx
BufferedSerial uart(A1,A0); //tx,rx
DigitalInOut ping(D11);
Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BufferedSerial xbee(D1, D0);
RpcDigitalOut myled3(LED1,"myled3");
RpcDigitalOut myled2(LED2,"myled2");
DigitalIn encoder0(D10);
DigitalIn encoder1(D12);
Timer t;
Thread thread;
Thread thread2;
Ticker encoder_ticker;
BBCar car(pin5, pin6, servo_ticker);
volatile int count_new = 0;
volatile int last0 = 0;
volatile int last1 = 0;
volatile int steps0 = 0;
volatile int steps1 = 0;
volatile int pretheta = 0;
//void doLine(Arguments *in, Reply *out);
//RPCFunction rpcLine(&doLine, "doLine");
//void doApriltag(Arguments *in, Reply *out);
//RPCFunction rpcApriltag(&doApriltag, "doApriltag");
Thread Linethread;
Thread Apriltagthread;
//void doParking(Arguments *in, Reply *out);
//RPCFunction rpcParkiing(&doParking, "doParking");
//void doBlock(Arguments *in, Reply *out);
//RPCFunction rpcBlock(&doBlock, "doBlock");
Thread Blockthread;
Thread Parkingthread;
volatile int pingping = 0;
volatile int position1 = 34, position2 = 38; //, position1, position2 are defined by the distance between apriltag and parking space
volatile int car_ping = 0;//position2 = 12.5-5.5
volatile float ping_block = 0;
volatile int ping_num = 0;

void ping_thread()
{
   float val;
   pc.set_baud(9600);
   //xbee.set_baud(9600);
   //xbee.write("hello", 5);
   while(1) {

      ping.output();
      ping = 0; wait_us(200);
      ping = 1; wait_us(5);
      ping = 0; wait_us(5);

      ping.input();
      while(ping.read() == 0);
      t.start();
      while(ping.read() == 1);
      val = t.read();
      ping_block = val*17700.4f-1;
      if (pingping == 1) {
         car_ping = int(val*17700.4f-1);
         ping_num = 3;
         printf("Ping=%lf\n", val*17700.4f-1);
         pingping = 0;
      }
      t.stop();
      t.reset();

      ThisThread::sleep_for(1s);
   }
}

void encoder_control() {
    int value0 = encoder0;
    if (!last0 && value0) steps0++;
    last0 = value0;
    int value1 = encoder1;
    if (!last1 && value1) steps1++;
    last1 = value1;
    count_new++;
}

void doeverything()
{
   uart.set_baud(9600);
   xbee.set_baud(9600);
   char buf[256];
   int haveread = 0;
   int double_check = 0;
   int enable = 1;
   while (ping_num < 4) {
      if (ping_num == 0) {
         memset(buf, 0, 256);
         int i = 0;
         int end = 0;
         int mark;
         int y = 0;
         char open[1];
         open[0] = 'L';
         uart.write(open, sizeof(open));
         count_new = 0;
         int nocount = 0;
         if (ping_block < 25) {
            ping_num = 1;
            char xopen[1];
            xopen[0] = 'L';
            xbee.write(xopen, sizeof(xopen));
            car.stop();
            ThisThread::sleep_for(50ms);
         }
         while (end == 0 && i < 256 && count_new < 100 && ping_block > 25) {
            if(uart.readable()){
               char recv[1];
               uart.read(recv, sizeof(recv));
               y = 1;
               pc.write(recv, sizeof(recv));
               buf[i] = recv[0];
               if (recv[0] == '\n') {
                  end = 1;
                  mark = i;
                  haveread++;
                  break;
               }
               i++;
            }
            //printf("read\n");
            if (count_new == 99 && y == 0) {
               uart.write(open, sizeof(open));
               count_new = 0;
               printf("no\n");
               nocount++;
               if (nocount < 10) {
                  car.goStraight(-5);
                  ThisThread::sleep_for(50ms);
               }
            }
         }
         int theta = 0, x = 0;
         int k = 0;
         for (k = 0; buf[k] != ','; k++) {
            theta = 10*theta;
            if (buf[k] == 0) break;
            theta += buf[k] - '0';
         }
         k++;
         for (; buf[k] != '\n'; k++) {
            x = 10*x;
            if (buf[k] == 0) break;
            x += buf[k] - '0';
         }
         //printf("%d\n", theta);
         //printf("x1 = %d, y1 = %d, x2 = %d, y2 = %d\n", x1, y1, x2, y2);
         //ThisThread::sleep_for(1000ms);
         if (haveread >= 2) {
            //printf("x1 = %d, y1 = %d, x2 = %d, y2 = %d\n", x1, y1, x2, y2);
            int angle = 0;
            if (x > 80) {
               car.turn(-50, 0.5);
               ThisThread::sleep_for(500ms);
            } else if (x < 60) {
               car.turn(-50, -0.5);
               ThisThread::sleep_for(500ms);
            } else if (pretheta > 90) {
               angle = 180-pretheta;
               if (angle > 30) { 
                  car.turn(-50, -0.5);
                  ThisThread::sleep_for(500ms);
               }else {
                  car.goStraight(-50);
                  ThisThread::sleep_for(1s);
               }
            } else {
               if (angle > 30) { 
                  car.turn(-50, 0.5);
                  ThisThread::sleep_for(500ms);
               }else {
                  car.goStraight(-40);
                  ThisThread::sleep_for(1s);
               }
            }
            car.stop();
            ThisThread::sleep_for(1s);
            pretheta = theta;
         } else if (haveread == 1) {
            pretheta = theta;
            car.goStraight(-40);
            ThisThread::sleep_for(1s);
         }
      } else if (ping_num == 1) {
         steps0 = 0;
         car.turn(-20, 0.1);
         while (steps0*6.5*3.14/32 < 10.5*2*3.14/4) {
            ThisThread::sleep_for(100ms);
         }
         steps1 = 0;
         car.turn(-100, -0.3);
         while (steps1*6.5*3.14/32 < 10.5*2*3.14*170/270) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         ThisThread::sleep_for(50ms);
         count_new = 0;
         while (count_new < 100) ping_num = 2;
         char xopen[1];
         xopen[0] = 'B';
         xbee.write(xopen, sizeof(xopen));
      } else if (ping_num == 2) {
         printf("A");
         memset(buf, 0, 256);
         int i = 0;
         int end = 0;
         int haveread2 = 0;
         char open[1];
         int y = 0;
         //car.stop();
         //ThisThread::sleep_for(100ms);
         open[0] = 'A';
         uart.write(open, sizeof(open));
         count_new = 0;
         while (haveread2 == 0 && end == 0 && enable == 1 && i < 256 && count_new < 100) {   // read from openmv
            if(uart.readable()){
               char recv[1];
               uart.read(recv, sizeof(recv));
               pc.write(recv, sizeof(recv));
               y = 1;
               buf[i] = recv[0];
               //printf("%c\n", buf[i]);
               if (recv[0] == '\n') {
                  end = 1;
                  haveread2 = 1;
                  break;
               }
               i++;
            }
         
            if (count_new == 99 && y == 0) {
               uart.write(open, sizeof(open));
               count_new = 0;
            }
         }
         if (enable == 0) {
            pingping = 1;
            enable = 2;
            char xopen[1];
            xopen[0] = 'A';
            xbee.write(xopen, sizeof(xopen));
         }
      
         double tx = 0, ty = 0, tz = 0;
         double txx = 0, tyy = 0, tzz = 0;
         double rx = 0, ry = 0, rz = 0;
         double rxx = 0, ryy = 0, rzz = 0;
         double goal_distance = 0, goal_angle = 0;
         int w = 0;
         int minus = 0;
         for (w = 0; buf[w] != '.'; w++) {
            tx = 10*tx;
            if (buf[w] == 0) break;
            if (buf[w] == '-') minus = 1;
            else tx += buf[w] - '0';
         }
         w++;
         for (; buf[w] != ','; w++) {
            txx = 10*txx;
            if (buf[w] == 0) break;
            txx += buf[w] - '0';
         }
         tx += txx/1000000;
         if (minus == 1) tx = -tx;
         w++;
         minus = 0;
         for (; buf[w] != '.'; w++) {
            ty = 10*ty;
            if (buf[w] == 0) break;
            if (buf[w] == '-') minus = 1;
            else ty += buf[w] - '0';
         }
         w++;
         for (; buf[w] != ','; w++) {
            tyy = 10*tyy;
            if (buf[w] == 0) break;
            tyy += buf[w] - '0';
         }
         ty += tyy/1000000;
         if (minus == 1) ty = -ty;
         w++;
         minus = 0;
         for (; buf[w] != '.'; w++) {
            tz = 10*tz;
            if (buf[w] == 0) break;
            if (buf[w] == '-') minus = 1;
            else tz += buf[w] - '0';
         }
         w++;
         for (; buf[w] != ','; w++) {
            tzz = 10*tzz;
            if (buf[w] == 0) break;
            tzz += buf[w] - '0';
         }
         tz += tzz/1000000;
         if (minus == 1) tz = -tz;
         w++;
         for (; buf[w] != '.'; w++) {
            rx = 10*rx;
            if (buf[w] == 0) break;
            rx += buf[w] - '0';
         }
         w++;
         for (; buf[w] != ','; w++) {
            rxx = 10*rxx;
            if (buf[w] == 0) break;
            rxx += buf[w] - '0';
         }
         rx += rxx/1000000;
         w++;
         for (; buf[w] != '.'; w++) {
            ry = 10*ry;
            if (buf[w] == 0) break;
            ry += buf[w] - '0';
         }
         w++;
         for (; buf[w] != ','; w++) {
            ryy = 10*ryy;
            if (buf[w] == 0) break;
            ryy += buf[w] - '0';
         }
         ry += ryy/1000000;
         w++;
         for (; buf[w] != '.'; w++) {
            rz = 10*rz;
            if (buf[w] == 0) break;
            rz += buf[w] - '0';
         }
         w++;
         for (; buf[w] != '\n'; w++) {
            rzz = 10*rzz;
            if (buf[w] == 0) break;
            rzz += buf[w] - '0';
         }
         rz += rzz/1000000;
         //printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tx, ty, tz, rx, ry, rz);
         if (haveread2 == 1) {
            printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tx, ty, tz, rx, ry, rz);
            if (double_check >= 1) {
               goal_distance = sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2))*2.8;
               goal_angle = ry;
               int angle_diff = 0;
               if (goal_angle > 0 && goal_angle < 180) {
                  car.turn(-20, 0.1);
                  angle_diff = (goal_angle - 0);
                  steps0 = 0;
                  while (steps0*6.5*3.14/32 < 10.5*2*3.14*(90 - angle_diff-5)/360) {
                     ThisThread::sleep_for(100ms);
                  }
                  steps0 = 0;
                  car.goStraight(-20);
                  while (steps0*6.5*3.14/32 < goal_distance*sin(angle_diff)-22) {
                     ThisThread::sleep_for(100ms);
                  }
                  steps1 = 0;
                  car.turn(-20, -0.1);
                  while (steps1*6.5*3.14/32 < 10.5*2*3.14*110/360) {
                     ThisThread::sleep_for(100ms);
                  }
               } else if (goal_angle > 180 && goal_angle < 360) {
                  car.turn(-20, -0.05);
                  steps1 = 0;
                  angle_diff = 360 - goal_angle;
                  while (steps1*6.5*3.14/32 < 10.5*2*3.14*(90 - angle_diff)/360) {
                     ThisThread::sleep_for(100ms);
                  }
                  steps0 = 0;
                  car.goStraight(-20);
                  while (steps0*6.5*3.14/32 < goal_distance*sin(angle_diff)-22) {
                     ThisThread::sleep_for(100ms);
                  }
                  steps0 = 0;
                  car.turn(-20, 0.05);
                  while (steps0*6.5*3.14/32 < 10.5*2*3.14*110/360) {
                     ThisThread::sleep_for(100ms);
                  }
               }
               car.stop();
               ThisThread::sleep_for(50ms);
               enable = 0;
            } else {
               double_check++;
            }
            /*car.goStraight(-20);
            while (steps0*6.5*3.14/32 < goal_distance) {
               ThisThread::sleep_for(100ms);
            }
            car.stop();*/
            
         }   
      } else if (ping_num == 3) {
         int d1 = position1 - 5;
         int d2 = position2 - car_ping - 14;
         // for East
         printf("%d, %d\n", d1, d2);
         car.goStraight(20);
         steps0 = 0;
         while(steps0*6.5*3.14/32 < d2+11.5) {
            ThisThread::sleep_for(100ms);
         }
         steps0 = 0;
         steps1 = 0;
         car.turn(20, -0.1);
         while (steps1*6.5*3.14/32 < 11*2*3.14*85/360) {
            ThisThread:: sleep_for(100ms);
         }
         steps1 = 0;
         steps0 = 0;
         car.goStraight(20);
         while(steps0*6.5*3.14/32 < d1+3.5) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         ping_num++;
         char xopen[1];
         xopen[0] = 'P';
         xbee.write(xopen, sizeof(xopen));
      }
   }
   car.stop();
}

int main(){
   
   thread.start(ping_thread);
   thread2.start(doeverything);
   //char buf[256], outbuf[256];
   double pwm_table0[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table0[] = {-16.741, -16.263, -15.147, -11.560, -5.182, 0.080, 6.378, 12.197, 15.227, 16.263, 16.742};
   double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table1[] = {-15.944, -15.546, -14.669, -11.958, -6.139, 0.000, 5.022, 11.002, 14.350, 15.546, 16.024};
   car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);
   encoder_ticker.attach(&encoder_control, 0.01);

}
