package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="BIL")
public class BILTeleOp extends TeleopCommon {

	boolean xPressed;
	boolean bPressed;
	ArmControl arm;

	public BILTeleOp() {}

	@Override
	public void init() {
		//Initializes all robot hardware parts
		robot.init(hardwareMap);
		setDriveMode(DriveMode.GTA);
		setMaxAcceleration(1.0);
		setMaxDeceleration(3.0);
		setDrivingMotors(robot.motorFrontLeft, robot.motorFrontRight, robot.motorBackLeft, robot.motorBackRight);
		arm = new ArmControl(robot.motorArm,robot.switchArm,0,0,0);
	}

	@Override
	public void loop() {
		updateTiming();
		setMotorSpeed(robot.motorLift, (gamepad1.dpad_up ? 1.0 : 0.0) - (gamepad1.dpad_down ? 1.0 : 0.0));
		//setMotorSpeed(robot.motorArm, (gamepad1.dpad_left ? 1.0 : 0.0) - (gamepad1.dpad_right ? 1.0 : 0.0));
		arm.updateArm((gamepad1.dpad_left ? 1.0 : 0.0) - (gamepad1.dpad_right ? 1.0 : 0.0));
		updateDriving();

		if(gamepad1.dpad_up && robot.switchTop.getState()){
            setMotorSpeed(robot.motorLift, 0.5);
        }else if(gamepad1.dpad_down && robot.switchBottom.getState()){
		    setMotorSpeed(robot.motorLift, -0.5);
        }else{
		    setMotorSpeed(robot.motorLift, 0);
        }

		if(gamepad1.x && !xPressed){
			robot.servoBlueGrab.setPosition(robot.servoBlueGrab.getPosition() > 0.5 ? 0.0 : 1.0);
			xPressed = true;
		}else if(!gamepad1.x){
			xPressed = false;
		}

		if(gamepad1.b && !bPressed){
			robot.servoRedGrab.setPosition(robot.servoRedGrab.getPosition() > 0.5 ? 0.0 : 1.0);
			bPressed = true;
		}else if(!gamepad1.b){
			bPressed = false;
		}

		robot.servoDeploy.setPosition(robot.servoDeploy.getPosition() + (gamepad1.a ? 0.01 : 0) - (gamepad1.y ? 0.01 : 0));
		telemetry.addData("Time passed", String.format("%f", time));
		telemetry.addData("Lift speed", String.format("%f", robot.motorLift.getPower()));
		telemetry.addData("Arm speed", String.format("%f", robot.motorArm.getPower()));
		telemetry.addData("Arm angle", String.format("%f", robot.motorArm.getPower()));
		telemetry.addData("Deploy pos", String.format("%f", robot.servoDeploy.getPosition()));
		telemetry.addData("Release pos", String.format("%f", robot.servoRelease.getPosition()));
		telemetry.addData("RedGrab pos", String.format("%f", robot.servoRedGrab.getPosition()));
		telemetry.addData("BlueGrab pos", String.format("%f", robot.servoBlueGrab.getPosition()));
		displaySpeedData();
		telemetry.update();
	}

	@Override
	public void stop() {

	}
}