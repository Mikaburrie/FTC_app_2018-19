package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Color.*;


/**
 * Created by lpane on 12/19/2017.
 */

@Autonomous(name="Jewel Red", group="BIL")
public class BILJewelRed extends BILAutonomousCommon{

    @Override public void runOpMode() throws InterruptedException {
        boolean rightMovement = false;
        boolean leftMovement = false;

        robot.init(hardwareMap);
        loadObjects();

        robot.colorSensor.enableLed(true);

        waitForStart();

        robot.leftGrabber.setPosition(0.8);

        robot.jewelArm.setPosition(0.0);

        delay(2000);

        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();

        Color left = detectLeft();

        if(left == RED) {
            setDriveMotors(0.5,0.5,-0.5,-0.5);
        } else if(left == BLUE) {
            setDriveMotors(-0.5,-0.5,0.5,0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);

        robot.jewelArm.setPosition(0.5);

        delay(500);

        if(left == RED) {
            setDriveMotors(-0.5,-0.5,0.5,0.5);
        } else if(left == BLUE) {
            setDriveMotors(0.5,0.5,-0.5,-0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);
        //setDriveMotors(0.5, -0.5, -0.5, 0.5);

        delay(2000);
    }
}