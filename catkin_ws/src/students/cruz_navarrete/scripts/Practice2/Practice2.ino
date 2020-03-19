/*
 * Integrantes del equipo:
 * Cruz Navarrete Victor Jesus
 * Hernández Martínez Cinthia
 * Gómez Castro Francisco Joel
 */

#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

void Callback_message_blink( const std_msgs::Int32& toggle_msg){
  
  int counter = 0;
  
  while(toggle_msg.data >= 0)
      {
          if(toggle_msg.data > 5000)
          {
            digitalWrite(13, HIGH-digitalRead(13));
            delay(toggle_msg.data/10);
            digitalWrite(13, HIGH-digitalRead(13));
            delay(toggle_msg.data/10);  
            counter++;
            if(counter > 2)
            {
              counter = 0;
              break;  
            }
          }
          else if(toggle_msg.data >= 0 && counter <= 2000)
          {
              digitalWrite(13, HIGH-digitalRead(13));
              delay(toggle_msg.data/10);
              digitalWrite(13, HIGH-digitalRead(13));
              delay(toggle_msg.data/10);  
              counter++;
              if(counter > 9)
              {
                counter = 0;
                break;  
              }
              
          }
          else if(toggle_msg.data > 2000 && counter <= 5000)
          {
              digitalWrite(13, HIGH-digitalRead(13));
              delay(toggle_msg.data/10);
              digitalWrite(13, HIGH-digitalRead(13));
              delay(toggle_msg.data/10);  
              counter++;
              if(counter > 6)
              {
                counter = 0;
                break;  
              }
          }  
      }
}


ros::Subscriber<std_msgs::Int32> sub("/period_ms", &Callback_message_blink );

void setup() {
  // put your setup code here, to run once:
  //initialize digital pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  // put your main code here, to run repeatedly:
 
  nh.spinOnce();
  delay(1);
}
