package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created on 1/7/2017 by Mika.
 */
public abstract class BILAutonomousCommon extends LinearOpMode {
    public BILRobotHardware robot = new BILRobotHardware();
    public ElapsedTime time = new ElapsedTime();

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    VuforiaTrackables imageTargets;
    VuforiaTrackable imageTemplate;
    //speed is in encoder ticks per second

    public enum Color {
        RED, BLUE, UNKNOWN
    }

    public enum Side {
        LEFT, MIDDLE, RIGHT, UNKNOWN
    }

    public final static int ticksPerRotation = 538;
    public final static double wheelDiameter = 4; //inches
    public final static double wheelCircumference = (wheelDiameter * Math.PI)/12; //circumference in feet
    public final static double ticksPerFoot = ticksPerRotation/wheelCircumference;
    public final static int driveTimeScalar = 3;
    public final static int ticksPerSecond = 2500;
    public final static double maxSpeed = ticksPerSecond/ticksPerFoot;

    MotionProfiler profiler = new MotionProfiler(maxSpeed, maxSpeed/2);
    DirectionHandler direction;

    public void initRobot() {
        robot.init(hardwareMap);
        direction = new DirectionHandler(robot.imu);
    }

    public void loadObjects() {
        this.vuforia = helper.initVuforia(false, 4);
        imageTargets = helper.loadTargets("RelicVuMark");
        imageTemplate = imageTargets.get(0);
        imageTemplate.setName("VuMarkTemplate");
    }

    /**
     * @param mode The run mode to set for all motors.
     */
    public void setAllMotorModes(DcMotor.RunMode mode) {
        robot.motorFrontRight.setMode(mode);
        robot.motorBackRight.setMode(mode);
        robot.motorFrontLeft.setMode(mode);
        robot.motorBackLeft.setMode(mode);
    }

    /**
     *
     * @param frontLeft Power for front left wheel.
     * @param backLeft Power for back left wheel.
     * @param frontRight Power for front right wheel.
     * @param backRight Power for back right wheel.
     */
    public void setDriveMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        robot.motorFrontLeft.setPower(frontLeft);
        robot.motorBackLeft.setPower(backLeft);
        robot.motorFrontRight.setPower(frontRight);
        robot.motorBackRight.setPower(backRight);
    }

    /**
     * @param power The power to set for all drive motors.
     */
    public void setAllDriveMotors(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    /**
     * @param distance How far the robot should travel (in feet).
     */
    public void driveDistance(double distance) throws InterruptedException {
        double direction = (distance < 0.0 ? -1.0 : 1.0);
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        time.reset();
        telemetry.addData("Profile info:", profiler.start(Math.abs(distance)));
        telemetry.update();
        while((opModeIsActive() && profiler.isRunning()) && Math.abs(robot.motorBackLeft.getCurrentPosition()) < Math.abs(ticksPerFoot * distance)) {
            setAllDriveMotors(profiler.getSpeed(time.seconds()) * direction);
        }

        //set all motors to 0
        setAllDriveMotors(0);
    }

    public void turnAngle(double angle, double speed) {
        direction.setTargetDirection(direction.getDirection() - angle);
        double offset;
        while(opModeIsActive() && Math.abs(offset = direction.getAngleToTarget()) > 2){
            double power = Math.max(offset/180 * speed, 0.1);
            setDriveMotors(power, power, -power, -power);
            telemetry.addData("", String.format("%f %f", offset, power));
            telemetry.update();
        }
        setAllDriveMotors(0);
    }

    public void driveByTime(double power, int milliseconds) throws InterruptedException {
        time.reset();
        setAllDriveMotors(power);
        while(time.milliseconds() < milliseconds) {
                idle();
        }
    }

    /**
     * @return If one or more motors are busy return true, otherwise false.
     */
    public boolean getAllMotorsBusy() {
        return (robot.motorFrontLeft.isBusy() || robot.motorBackLeft.isBusy() || robot.motorFrontRight.isBusy() || robot.motorBackRight.isBusy());
    }


    public void driveToTarget(VectorF translation){
        double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
        double zTrans = (double)translation.get(2);

        double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

        double leftSpeed = Range.clip((40 + degreesToTurn * 2) / 100, -Math.abs(zTrans) / 2000, Math.abs(zTrans) / 2000);
        double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
        setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
    }

    public void moveToImage(VuforiaTrackable target, BILVuforiaCommon helper) throws InterruptedException {
        boolean inFrontOfImage = false;
        while(!inFrontOfImage){
            VectorF translation = helper.getTargetTranslation(target);
            if(translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) { // 2 = z
                driveToTarget(translation);
                telemetry.addData("Translation X", translation.get(0));
                telemetry.addData("Translation Y", translation.get(1));
                telemetry.addData("Translation Z", translation.get(2));
            } else {
                if(translation != null){
                    inFrontOfImage = true;
                } else {
                    telemetry.addData(target.getName() + " Target", "not in view");
                }
            }
            telemetry.update();
            idle();
        }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)time.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        time.reset();
    }

    public Side detectImageSide() {
        time.reset();
        RelicRecoveryVuMark position = RelicRecoveryVuMark.UNKNOWN;

        imageTargets.activate();

        while(opModeIsActive() && !isStopRequested() && position == RelicRecoveryVuMark.UNKNOWN) {

            //if taking too long
            if(time.milliseconds() > 5000){
                return Side.UNKNOWN;
            }

            position = RelicRecoveryVuMark.from(imageTemplate);
            if (position != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", position);
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
            idle();
        }

        imageTargets.deactivate();

        if(position == RelicRecoveryVuMark.LEFT){
            return Side.LEFT;
        }else if(position == RelicRecoveryVuMark.CENTER){
            return Side.MIDDLE;
        }else{
            return Side.RIGHT;
        }
    }

    public void driveToPos(Side pos) {
        //aligning, etc.
    }

    public void parkSimple(Color teamColor) {
        if(teamColor == Color.RED) {
            setDriveMotors(0.5, 0.5, -0.5, -0.5);
        } else if(teamColor == Color.BLUE) {
            setDriveMotors(-0.5, -0.5, 0.5, 0.5);
        }

        delay(500);

        setAllDriveMotors(0);

        delay(1000);

        setAllDriveMotors(0.5);

        delay(1000);

        setAllDriveMotors(0);
    }

    /**
     *
     * @param milliseconds The amount of time to wait.
     */
    public void delay(long milliseconds) {
        time.reset();
        while(time.milliseconds() < milliseconds && !isStopRequested()){
            idle();
        }
    }
}