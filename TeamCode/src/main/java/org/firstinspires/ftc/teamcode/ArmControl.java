package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
* class to control arms by a PID loop
* made to be used with tetrix motors and encoders
* designed to be used with the 2018-2019 robot
* by Nick
*/
public class ArmControl {

    DcMotor armMotor;
    DigitalChannel limitSwitch;
    double topSpeed = 45.0; //in degrees/second for the arm
    double gearing = 9.0; // how many times farther does the arm have to move than the motor
    double ticksPerDegree= 1440/360.0;
    private ElapsedTime updateTime = new ElapsedTime();
    private double deltaTime = 0;
    private double pValue;
    private double iValue;
    private double dValue;
    private int error;
    private int oldError;
    private int errorIntergral;
    private int targetPos;
    boolean isHoming;

    public ArmControl(DcMotor armMotor,DigitalChannel endSwitch, double p,double i, double d){
        this.armMotor = armMotor;
        pValue = p;
        limitSwitch = endSwitch;
        dValue = d;
        iValue = i;
        isHoming = false;
        resetPosition();
    }
    public void updateTiming() {
        deltaTime = updateTime.seconds();
        updateTime.reset();
    }
    private void resetPosition(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        error =0;
        oldError=0;
        errorIntergral =0;
        targetPos=0;
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void updateArm(double input){
        updateTiming();
        targetPos = (int)(topSpeed*deltaTime*input*ticksPerDegree*gearing) + targetPos;
        if(limitSwitch.getState()&&input<0){//if the switch is true (this needs to be checked)
            resetPosition();
            isHoming = false;
            return;
        }
        if(isHoming&& input == 0){
            targetPos = (int)(-topSpeed*deltaTime*ticksPerDegree*gearing) + targetPos;
        }else if(input!=0){
            isHoming = false;
        }
        error = targetPos-armMotor.getCurrentPosition();
        armMotor.setPower(getArmPower());
        oldError=error;

    }
    public double getArmPower(){
        double armPower;
        errorIntergral +=error*deltaTime;
        armPower = error*pValue+((error-oldError)/deltaTime)*dValue+errorIntergral*iValue; // setting arm power to a value using pid
        if(Math.abs(armPower)>1){
            armPower=(Math.pow(armPower,2))/Math.abs(armPower);//if armPower is greater or lesser than 1 or -1 respectivly set armPower to 1 or -1 respectivly
        }
        return armPower;
    }
    public void updateArm(){//this is for autonomous
        updateTiming();
        if(limitSwitch.getState()){//if the switch is true (this needs to be checked)
            resetPosition();
            isHoming = false;
            return;
        }
        if(isHoming) {
            targetPos = (int) (-topSpeed * deltaTime * ticksPerDegree * gearing) + targetPos;
        }
        error = targetPos-armMotor.getCurrentPosition();
        armMotor.setPower(getArmPower());
        oldError=error;
    }
    public void setArmMoveAngle(double angle){//in degrees, as view from the left side of the robot
        targetPos = targetPos + (int)(angle*ticksPerDegree);//should i switch target with get current?
    }
    public void setIsHoming(boolean input){
        isHoming = input;
    }
    public double getAngle(){
        return armMotor.getCurrentPosition()/ticksPerDegree;
    }

}
