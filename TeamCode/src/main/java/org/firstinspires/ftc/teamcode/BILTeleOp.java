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
		setMaxAcceleration(0.1);
		setDrivingMotors(robot.motorFrontLeft, robot.motorFrontRight, robot.motorBackLeft, robot.motorBackRight);
	}

	@Override
	public void loop() {
		updateDriving();
	}

	@Override
	public void stop() {

	}
}