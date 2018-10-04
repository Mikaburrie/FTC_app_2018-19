package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;

/**
 * Created by lpane on 1/2/2018.
 */


@Autonomous(name="Cryptobox Align", group="BIL")
public class BILCryptoAllign extends BILAutonomousCommon {

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        CryptoboxDetector detector = new CryptoboxDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();

        if (detector.getCryptoBoxCenterPosition() > 175) {
            setDriveMotors(-0.5, 0.5, 0.5, -0.5);
        } else {
            setDriveMotors(0.5, -0.5, -0.5, 0.5);
        }

        time.reset();

        delay(250);

        setAllDriveMotors(0);
    }
}