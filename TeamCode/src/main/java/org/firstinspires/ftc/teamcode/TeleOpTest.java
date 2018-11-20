package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="TeleOpTest", group="BIL")
public class TeleOpTest extends TeleopCommon {


    /**
     * Constructor
     */
    public TeleOpTest() {}
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        //Initializes all robot hardware parts
        robot.init(hardwareMap);
        setDrivingMotors(robot.motorFrontLeft, robot.motorFrontRight, robot.motorBackLeft, robot.motorBackRight);
        setDriveMode(DriveMode.GTA);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        updateDriving(0.8);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

}