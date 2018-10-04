package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="TeleOp", group="BIL")
public class BILTeleOp extends OpMode {

	BILRobotHardware robot = new BILRobotHardware(); // use the class created to define a Pushbot's hardware

	double frontRight;
	double frontLeft;
	double backRight;
	double backLeft;

	double throttleY;
	double throttleX;
	double turning;
	double liftSpeed;
	double glyphGatherer;

	double liftPitch = 0;

	double expo;

	boolean directionRobot = true;
	double maxSpeed = 0.4;
	BILTeleOpJoystick bilTeleOpJoystick;
	/**
	 * Constructor
	 */
	public BILTeleOp() {
		bilTeleOpJoystick = new BILTeleOpJoystick();
	}
	/*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
	@Override
	public void init() {

		//Initializes all robot hardware parts
		robot.init(hardwareMap);
	}

	/*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
	@Override
	public void loop() {
		getJoystickInput();

		scaleJoystickInput();

		//    getGamepadInputs();

		setMotorSpeeds();

		//telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("F/R", "direction:  " + String.format("%b",directionRobot ));
		telemetry.addData("FrontLeft Power", String.format("%.2f", frontLeft));
		telemetry.addData("BackLeft Power", String.format("%.2f", backLeft));
		telemetry.addData("FrontRight Power", String.format("%.2f", frontRight));
		telemetry.addData("BackRight Power", String.format("%.2f", backRight));
		telemetry.addData("lift Power", String.format("%.2f", liftSpeed));
		telemetry.update();
	}

	/*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
	@Override
	public void stop() {

	}

	protected void getJoystickInput() {
		// throttleY: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// throttleX: left_stick_x ranges from -1 to 1, where -1 is full left, and
		// 1 is full right
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		throttleX = gamepad1.left_stick_x;
		throttleY = gamepad1.left_stick_y;
		turning = -gamepad1.right_stick_x;
		glyphGatherer = gamepad2.right_stick_y;
		liftSpeed = -gamepad2.left_stick_y;
	}

	protected void scaleJoystickInput() {
		if(gamepad1.left_trigger == 0.0) {
			expo = 2.0;
		}
		else {
			expo = 1.1;
		}

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		throttleY = bilTeleOpJoystick.normalizeSpeed(throttleY, expo, maxSpeed);
		throttleX = bilTeleOpJoystick.normalizeSpeed(throttleX, expo, maxSpeed);
		turning = bilTeleOpJoystick.normalizeSpeed(turning, expo, maxSpeed);
		liftSpeed = bilTeleOpJoystick.normalizeSpeed(liftSpeed, 2.0, maxSpeed);
	}

	protected void getMeccanumMotorSpeeds(double leftX, double leftY, double rightX) {
		frontRight = leftY - leftX - rightX;
		backRight = leftY + leftX - rightX;
		frontLeft = leftY + leftX + rightX;
		backLeft = leftY - leftX + rightX;

		frontRight = Range.clip(frontRight, -maxSpeed, maxSpeed);
		backRight = Range.clip(backRight, -maxSpeed, maxSpeed);
		frontLeft = Range.clip(frontLeft, -maxSpeed, maxSpeed);
		backLeft = Range.clip(backLeft, -maxSpeed, maxSpeed);
	}

	protected void setMotorSpeeds() {
		getMeccanumMotorSpeeds(throttleX, throttleY, turning);

		// write the values to the motors
		robot.motorFrontLeft.setPower(frontLeft);
		robot.motorBackLeft.setPower(backLeft);
		robot.motorFrontRight.setPower(frontRight);
		robot.motorBackRight.setPower(backRight);
		robot.motorLift.setPower(liftSpeed);
		robot.gatherRight.setPower(glyphGatherer);
		robot.gatherLeft.setPower(glyphGatherer);
		robot.motorLift.setPower(liftSpeed);
	}

	protected void getGamepadInputs() {

		if(gamepad1.x) robot.liftPitch.setPosition(0);
		else robot.liftPitch.setPosition(0.70);

		if(gamepad1.dpad_up)
			liftSpeed = .75;
		else
			liftSpeed = 0;
		if(gamepad1.dpad_down)
			liftSpeed = -.75;
		else
			liftSpeed = 0;
	}

}