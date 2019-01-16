package org.firstinspires.ftc.teamcode;

/**
 * Created by nill on 1/9/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Color.*;
import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Side.*;

@Autonomous(name="BILTest")
public class BILTestMode extends BILAutonomousCommon
{
    @Override public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        double speed = 0.3;

        while(opModeIsActive()) {
            if (gamepad1.x) {
                turnAngle(-90, speed);
            } else if (gamepad1.b) {
                turnAngle(90, speed);
            }
        }
    }
}
