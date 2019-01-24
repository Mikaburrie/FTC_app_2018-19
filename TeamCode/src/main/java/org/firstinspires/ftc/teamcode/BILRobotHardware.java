package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public DcMotor motorArm;
    public Servo servoDeploy;
    public double deployMin = 0.14;
    public double deployMax = 0.84;
    public Servo servoRelease;
    public double releaseMin;
    public double releaseMax;
    public Servo servoRedGrab;
    public double redGrabMin;
    public double redGrabMax;
    public Servo servoBlueGrab;
    public double blueGrabMin;
    public double blueGrabMax;

    public DigitalChannel switchBottom;
    public DigitalChannel switchTop;

    BNO055IMU imu;

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
        motorFrontRight = getMotor("Front_Right", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight = getMotor("Back_Right", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft = getMotor("Front_Left", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft = getMotor("Back_Left", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift = getMotor("Lift_Motor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm = getMotor("Arm_Motor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);

        servoDeploy = hwMap.servo.get("Deploy_Servo");
        servoRelease = hwMap.servo.get("Release_Servo");
        servoRedGrab = hwMap.servo.get("Red_Grab_Servo");
        servoBlueGrab = hwMap.servo.get("Blue_Grab_Servo");

        switchBottom = hwMap.digitalChannel.get("Limit_Bottom");
        switchTop = hwMap.digitalChannel.get("Limit_Top");
        switchBottom.setMode(DigitalChannel.Mode.INPUT);
        switchTop.setMode(DigitalChannel.Mode.INPUT);

        initIMU();
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private DcMotor getMotor(String name, DcMotor.Direction direction, DcMotor.RunMode mode) {
        DcMotor motor = hwMap.dcMotor.get(name);
        motor.setDirection(direction);
        motor.setMode(mode);
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}