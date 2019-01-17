package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="BIL")
public class BILTeleOp extends TeleopCommon {

	public BILTeleOp() {}

	@Override
	public void init() {
		//Initializes all robot hardware parts
		robot.init(hardwareMap);
		setDriveMode(DriveMode.GTA);
		setMaxAcceleration(1.0);
		setMaxDeceleration(3.0);
		setDrivingMotors(robot.motorFrontLeft, robot.motorFrontRight, robot.motorBackLeft, robot.motorBackRight);
	}

	@Override
	public void loop() {
		updateTiming();
		setMotorSpeed(robot.motorLift, (gamepad1.dpad_up ? 1.0 : 0.0) - (gamepad1.dpad_down ? 1.0 : 0.0));
		setMotorSpeed(robot.motorArm, (gamepad1.dpad_left ? 1.0 : 0.0) - (gamepad1.dpad_right ? 1.0 : 0.0));
		updateDriving();
		robot.servoDeploy.setPosition(robot.servoDeploy.getPosition() + (gamepad1.a ? 0.01 : 0) - (gamepad1.b ? 0.01 : 0));
		robot.servoRelease.setPosition(robot.servoRelease.getPosition() + (gamepad1.x ? 0.01 : 0) - (gamepad1.y ? 0.01 : 0));
		robot.servoRedGrab.setPosition(robot.servoRedGrab.getPosition() + (gamepad1.right_trigger > 0.5 ? 0.01 : 0) - (gamepad1.right_bumper ? 0.01 : 0));
		robot.servoBlueGrab.setPosition(robot.servoBlueGrab.getPosition() + (gamepad1.left_trigger > 0.5 ? 0.01 : 0) - (gamepad1.left_bumper ? 0.01 : 0));
		telemetry.addData("Time passed", String.format("%f", time));
		telemetry.addData("Lift speed", String.format("%f", robot.motorLift.getPower()));
		telemetry.addData("Arm speed", String.format("%f", robot.motorArm.getPower()));
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