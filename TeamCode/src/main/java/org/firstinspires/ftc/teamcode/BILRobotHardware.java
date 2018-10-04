package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created on 10/20/2016 by Mika.
 */
public class BILRobotHardware {
    /* Public OpMode members. */
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorLift;

    public LightSensor lightSensor;
    public GyroSensor gyroSensor;
    public ColorSensor colorSensor;

    public DcMotor gatherRight;
    public DcMotor gatherLeft;
    public Servo liftPitch;

    //Compatibility
    public Servo jewelArm;
    public Servo leftGrabber;
    public double pusherRight = 0;
    public double pusherLeft = 0;
    public double pusherMiddle = 0;

    public double liftHolderStart = 0.2;
    public int liftHolderRelease = 1;
    public double rightGripperMaxRight = 0.0;
    public double leftGripperMaxLeft = 0.0;
    public double jewelArmStartPosition = 0.5;

    public Servo pusher; //for compatibility

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public BILRobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        //Left_Front, Left_Back, Right_Front, Right_Back
        motorFrontRight = hwMap.dcMotor.get("Right_Front");
        motorBackRight = hwMap.dcMotor.get("Right_Back");
        motorFrontLeft = hwMap.dcMotor.get("Left_Front");
        motorBackLeft = hwMap.dcMotor.get("Left_Back");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //Lift mechanisms
        motorLift = hwMap.dcMotor.get("Lift");
        liftPitch = hwMap.servo.get("Lift_Pitch");

        //Glyph sweeper
        gatherRight = hwMap.dcMotor.get("Glyph_Gatherer_Right");
        gatherLeft = hwMap.dcMotor.get("Glyph_Gatherer_Left");
        gatherLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorLift.setPower(0);

        // Set all motors to run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to brake when set to zero power.
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
