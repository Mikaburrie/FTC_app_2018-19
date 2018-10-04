package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="DemoOp", group="BIL")
public class BILDemoOp extends OpMode {

    BILRobotHardware robot = new BILRobotHardware(); // use the class created to define a Pushbot's hardware

    double frontRight;
    double frontLeft;
    double backRight;
    double backLeft;

    double throttleY;
    double throttleX;
    double turning;
    double liftSpeed;
    double gatherSpeed;
    double liftPos = 0.9;

    double expo = 2;

    double maxSpeed = 0.7;
    double liftUp = 0.3;
    double liftDown = 0.9;

    BILTeleOpJoystick bilTeleOpJoystick;
    /**
     * Constructor
     */
    public BILDemoOp() {
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

        getGamepadInputs();

        setMotorSpeeds();

        //telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("FrontLeft Power", String.format("%.2f", frontLeft));
        telemetry.addData("BackLeft Power", String.format("%.2f", backLeft));
        telemetry.addData("FrontRight Power", String.format("%.2f", frontRight));
        telemetry.addData("BackRight Power", String.format("%.2f", backRight));
        telemetry.addData("lift Power", String.format("%.2f", liftSpeed));
        telemetry.addData("Pitch Power", String.format("%.2f", liftPos));
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
        gatherSpeed = gamepad2.right_stick_y;
        liftSpeed = gamepad2.left_stick_y;
    }

    protected void scaleJoystickInput() {
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        throttleY = bilTeleOpJoystick.normalizeSpeed(throttleY, expo, maxSpeed);
        throttleX = bilTeleOpJoystick.normalizeSpeed(throttleX, expo, maxSpeed);
        turning = bilTeleOpJoystick.normalizeSpeed(turning, expo, maxSpeed);
        gatherSpeed = bilTeleOpJoystick.normalizeSpeed(gatherSpeed, expo, maxSpeed);
        liftSpeed = bilTeleOpJoystick.normalizeSpeed(liftSpeed, expo, maxSpeed);
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

        robot.gatherRight.setPower(gatherSpeed);
        robot.gatherLeft.setPower(gatherSpeed);
        robot.motorLift.setPower(liftSpeed);
    }

    protected void getGamepadInputs() {

        if(gamepad2.right_trigger > 0.5){
            liftPos -= 0.01;
        } else {
            liftPos += 0.01;
        }

        if(liftPos > liftDown){
            liftPos = liftDown;
        } else if(liftPos < liftUp){
            liftPos = liftUp;
        }
        robot.liftPitch.setPosition(liftPos);

    }
}