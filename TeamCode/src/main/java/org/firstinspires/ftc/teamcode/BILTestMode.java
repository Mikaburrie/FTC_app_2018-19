package org.firstinspires.ftc.teamcode;

/**
 * Created by nill on 1/9/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Color.*;
import static org.firstinspires.ftc.teamcode.BILAutonomousCommon.Side.*;

@Autonomous(name="BIlTest")
public class BILTestMode extends BILAutonomousCommon
{
    @Override public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        driveDistance(1);
    }
}
