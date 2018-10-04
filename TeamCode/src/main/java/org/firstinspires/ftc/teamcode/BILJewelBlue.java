package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Color.BLUE;
import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Color.RED;


/**
 * Created by lpane on 12/19/2017.
 */


@Autonomous(name="Jewel Blue", group="BIL")
public class BILJewelBlue extends BILAutonomousCommon {

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.colorSensor.enableLed(true);

        // robot.colorSensor = hardwareMap.colorSensor.get("colorSensor");

        waitForStart();

        robot.jewelArm.setPosition(0.0);

        delay(2000);

        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();

        Color left = detectLeft();

        if(left == BLUE) {
            setDriveMotors(0.5,0.5,-0.5,-0.5);
        } else if(left == RED) {
            setDriveMotors(-0.5,-0.5,0.5,0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);

        robot.jewelArm.setPosition(0.5);

        delay(500);

        if(left == BLUE) {
            setDriveMotors(-0.5,-0.5,0.5,0.5);
        } else if(left == RED) {
            setDriveMotors(0.5,0.5,-0.5,-0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);
        //setDriveMotors(0.5, -0.5, -0.5, 0.5);

        delay(2000);
    }
}
