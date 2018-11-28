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

        robot.init(hardwareMap);

        waitForStart();

        driveDistance(1);
        delay(20000);
        driveDistance(4);
        delay(20000);
        driveDistance(10);
        delay(2000);
    }
}
