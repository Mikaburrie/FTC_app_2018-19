package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
* class to control arms
* made to be used with tetrix motors and encoders
* designed to be used with the 2018-2019 robot
* by Nick
*/
public class ArmControl {


    DigitalChannel limitSwitch;
    double gearing = 9.0; // how many times farther does the arm have to move than the motor
    double ticksPerDegree= 1440/360.0;
    private ElapsedTime updateTime = new ElapsedTime();
    double offset;
    double upperLimit=100;// all limit and slow values are in degrees
    double upperSlow = 80;
    double lowerLimit=0;
    double lowerSlow=20;

    public ArmControl(DigitalChannel endSwitch){
        limitSwitch=endSwitch;
        offset =0;
    }

    public double getMotorSpeed(boolean lower, boolean raise,int encValue){
        if(raise&&(!lower)&&limitSwitch.getState()) {//assuming limitSwitch is true when pressed
            offset = encValueToDegrees(encValue);
            return 0;
        }else if(!(raise^lower)){
            return 0;
        }else if(raise){
            if(encValueToDegrees(encValue)>upperLimit){
                return 0;
            }else{
                if(encValueToDegrees(encValue)>upperSlow){
                    return (upperLimit-encValueToDegrees(encValue))/(upperLimit-upperSlow);
                }else{
                    return 1;
                }
            }
        }else{
            if(encValueToDegrees(encValue)<lowerLimit){
                return 0;
            }else{
                if(encValueToDegrees(encValue)<lowerSlow){
                    return (lowerLimit-encValueToDegrees(encValue))/lowerSlow;
                }else{
                    return -1;
                }
            }
        }
    }
    public double encValueToDegrees(int encValue){
        return encValue/(ticksPerDegree*gearing);
    }



}
