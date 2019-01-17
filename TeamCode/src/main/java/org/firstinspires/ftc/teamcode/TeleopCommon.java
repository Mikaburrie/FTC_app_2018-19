package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class TeleopCommon extends OpMode {
    public BILRobotHardware robot = new BILRobotHardware();
    private DriveMode driveMode;
    private DcMotor Fl;
    private DcMotor Fr;
    private DcMotor Bl;
    private DcMotor Br;
    private double joyLX;
    private double joyLY;
    private double joyRX;
    private double joyRY;
    private double maxAcceleration;
    private double maxDeceleration;
    private ElapsedTime updateTime = new ElapsedTime();
    private double deltaTime = 0;

    public enum DriveMode {
        TANK,
        GTA,
        MECCANUM,
    };

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public void setDrivingMotors(DcMotor left, DcMotor right) {
        setDrivingMotors(left, right, left, right);
    }

    public void setDrivingMotors(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        Fl = frontLeft;
        Fr = frontRight;
        Bl = backLeft;
        Br = backRight;
    }

    public void setMaxAcceleration(double max) {
        maxAcceleration = max;
    }
    public void setMaxDeceleration(double max) {
        maxDeceleration = max;
    }

    public void updateTiming() {
        deltaTime = updateTime.seconds();
        updateTime.reset();
    }

    public void updateDriving() {
        updateDriving(1.0, 2, 0.05);
    }

    public void updateDriving(double maxSpeed) {
        updateDriving(maxSpeed, 2, 0.05);
    }

    public void updateDriving(double maxSpeed, double exponent, double deadband) {
        joyLX = gamepad1.left_stick_x;
        joyLY = gamepad1.left_stick_y;
        joyRX = gamepad1.right_stick_x;
        joyRY = gamepad1.right_stick_y;
        switch(driveMode) {
            case TANK:
                updateTank(maxSpeed, exponent, deadband);
            break;
            case GTA:
                updateGTA(maxSpeed, exponent, deadband);
            break;
            case MECCANUM:
                updateMeccanum(maxSpeed, exponent, deadband);
            break;
        }
    }

    private void updateTank(double maxSpeed, double exponent, double deadband) {
        setMotorSpeed(Fl, joyLY, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Fr, joyRY, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Bl, joyLY, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Br, joyRY, -maxSpeed, maxSpeed, exponent, deadband);
    }

    private void updateGTA(double maxSpeed, double exponent, double deadband) {
        setMotorSpeed(Fl, joyLY - joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Fr, joyLY + joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Bl, joyLY - joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Br, joyLY + joyRX, -maxSpeed, maxSpeed, exponent, deadband);
    }

    private void updateMeccanum(double maxSpeed, double exponent, double deadband) {
        setMotorSpeed(Fl, joyLY + joyLX + joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Fr, joyLY - joyLX - joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Bl, joyLY - joyLX + joyRX, -maxSpeed, maxSpeed, exponent, deadband);
        setMotorSpeed(Br, joyLY + joyLX - joyRX, -maxSpeed, maxSpeed, exponent, deadband);
    }

    public void setMotorSpeed(DcMotor motor, double value) {
        double diff = value - motor.getPower();
        if(Math.abs(value) > Math.abs(motor.getPower())){ //accelerating
            if(Math.abs(diff) > maxAcceleration * deltaTime){
                value = (diff < 0 ? -maxAcceleration : maxAcceleration) * deltaTime + motor.getPower();
            }
        }else{ //decelerate
            if(Math.abs(diff) > maxDeceleration * updateTime.seconds()){
                value = (diff < 0 ? -maxDeceleration : maxDeceleration) * deltaTime + motor.getPower();
            }
        }

        motor.setPower(value);
    }

    public void setMotorSpeed(DcMotor motor, double value, double min, double max, double exponent, double deadband) {
        double scaledValue = Math.pow(Math.abs(value) < deadband ? 0 : value, exponent) * (value < 0 ? -1 : 1);
        setMotorSpeed(motor, Range.scale(Range.clip(scaledValue, -1, 1), -1, 1, min, max));
    }

    public void setServoPosition(Servo servo, double value) {
        servo.setPosition(value);
    }

    public void setServoPosition(Servo servo, double value, double min, double max) {
        setServoPosition(servo, Range.clip(value, min, max));
    }

    public void displayJoystickData(){
        telemetry.addData("Left X", String.format("%f", joyLX));
        telemetry.addData("Left Y", String.format("%f", joyLY));
        telemetry.addData("Right X", String.format("%f", joyRX));
        telemetry.addData("Right Y", String.format("%f", joyRY));
    }

    public void displaySpeedData(){
        telemetry.addData("Front Left", String.format("%f", Fl.getPower()));
        telemetry.addData("Back Left", String.format("%f", Bl.getPower()));
        telemetry.addData("Front Right", String.format("%f", Fr.getPower()));
        telemetry.addData("Back Right", String.format("%f", Br.getPower()));
    }
}