package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by lpane on 1/2/2018.
 */


public class BILCryptoAllign extends BILAutonomousCommon {

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        //CryptoboxDetector detector = new CryptoboxDetector();

        //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //detector.enable();

        if (/*detector.getCryptoBoxCenterPosition()*/0 > 175) {
            setDriveMotors(-0.5, 0.5, 0.5, -0.5);
        } else {
            setDriveMotors(0.5, -0.5, -0.5, 0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);
    }
}