#include <Wire.h>
#include "pwm_actuator.h"
#include "iris_pinout.h"
#include "FreeSixIMU.h"
 
//Call the constructors
pwm_actuator collect_rotating = pwm_actuator();
pwm_actuator bin_linear = pwm_actuator();
pwm_actuator collect_linear = pwm_actuator();
FreeSixIMU imu = FreeSixIMU();

void setup()
{
    //Maxon pinout
    pinMode(MAXON_BRAKE, OUTPUT); //unset brake
    collect_rotating.init(
        MAXON_SPEED, MAXON_DIR, MAXON_ENABLE, //pins
        255, // extend_speed (forward)
        0, // retract_speedint (backwards)
        1 //brake_negate (enable negate)
        );
    pinMode(MAXON_HALL, INPUT);

    //Dispensation LA pinout
    bin_linear.init(
        DISP_SPEED, DISP_DIR, DISP_BRAKE, //pins
        255, // extend_speed (forward)
        255, // retract_speedint (backwards)
        0 //brake_negate (enable negate)
        );
    pinMode(DISP_THERMAL, INPUT);
    pinMode(DISP_HALL, INPUT);
    pinMode(DISP_POT, INPUT);
    pinMode(DISP_CURRENT, INPUT);
    digitalWrite(A0, HIGH);             //Set the pullup resistor

    //Paddle LA pinout
    collect_linear.init(
        ARM_SPEED, ARM_DIR, ARM_BRAKE, //pins
        127, // extend_speed (forward)
        127, // retract_speedint (backwards)
        0 //brake_negate (enable negate)
        );
    pinMode(ARM_THERMAL, INPUT);
    pinMode(ARM_POT, INPUT);
    pinMode(ARM_CURRENT, INPUT);
        
    //Serial communications
    Serial.begin(115200);               //ODroid Serial
    Serial1.begin(115200);              //Roboteq Serial

    //I2C communications
    Wire.begin();

    //Initialize IMU
    //imu.init();  

    Serial.println("Ready");
}

void loop()
{
    //input handling
    int commaIdx[4];
    int idx_start = 0;
    int idx_end = 0;
    int idx = 0;
    bool data = false;
    String str;
    //Robot Status
    float pose[10];
    
    //Wait for data to arrive (Format: "spd_l,spd_r,paddle_LA,bin_LA,maxon#!")
    while(Serial.available() > 0)
    {
        str = "";
        str = Serial.readStringUntil('!');
        idx_start = 0;
        idx_end = str.length();

        // Check we got good data
        data = str.endsWith("#");
    }
    

    if(data)
    {
        //Find the index of the commas
        commaIdx[0] = str.indexOf(',', idx_start);
        commaIdx[1] = str.indexOf(',', commaIdx[0]+1);
        commaIdx[2] = str.indexOf(',', commaIdx[1]+1);
        commaIdx[3] = str.indexOf(',', commaIdx[2]+1);

        //Handle motor commands
        String temp_str = roboteq_string(
            str.substring(commaIdx[0]+1, commaIdx[1]), //speed R
            str.substring(idx_start, commaIdx[0])  //speed L
            );
        Serial1.write(temp_str.c_str(), temp_str.length());
        Serial1.flush();
        
        //Handle collection LA commands
        if(str[commaIdx[2]-1] == '0')
        {
            int dist = analogRead(A14);
            /*//Potentiometer stop check
            if(dist > 190)
            {
                collect_linear.stop(); //Actuator fully extended
            }
            else
            {
                collect_linear.extend();
            }*/
            collect_linear.extend();

        }
        else if(str[commaIdx[2]-1] == '1')
        {
            collect_linear.stop();
        }
        else if(str[commaIdx[2]-1] == '2')
        {
            int dist = analogRead(A14);
            /*//Potentiometer stop check
            if(dist < 140)
            {
                collect_linear.stop();
            }
            else
            {
                collect_linear.retract();
            }*/
            collect_linear.retract();
        }

        //Handle bin LA commands
        if(str[commaIdx[3]-1] == '0')
        {
            /*//Potentiometer stop check
            if(analogRead(A0) < 100) //end switch beign pushed
            {
                bin_linear.stop();
            }
            else
            {
                bin_linear.retract();
            }*/
            bin_linear.retract();
        }
        else if(str[commaIdx[3]-1] == '1')
        {
            bin_linear.stop();
        }
        else if(str[commaIdx[3]-1] == '2')
        {
            bin_linear.extend();
        }

        //Handle collection rotating commands
        if(str[str.length()-2] == '1')
        {
            collect_rotating.forward();
        }
        else
        {
            collect_rotating.stop();
        }
        data = false;


        //Update the robot status
        //Format: "x, y, theta, x_error, y_error, theta_error, v_x, v_theta, disp_act, turbine_act, maxon"
        //imu.getEuler(pose); //Get IMU data //fills pose[0] pose[1] pose[2]
        pose[3] = 0; //servo_current;
        pose[4] = 0; //servo_destination;
        pose[5] = 0;
        pose[6] = 0;
        pose[7] = str[commaIdx[3]-1];
        pose[8] = str[commaIdx[2]-1];
        pose[9] = str[str.length()-2];
    
        //Create the string to send
        String status_str;
        for(int i=0;i<10;i++)
        {
          status_str += i?",":"";
          status_str += String(pose[0]);
        }
        status_str += "#!\n";
        Serial.write(status_str.c_str(), status_str.length());
        Serial.flush();
    }
}
