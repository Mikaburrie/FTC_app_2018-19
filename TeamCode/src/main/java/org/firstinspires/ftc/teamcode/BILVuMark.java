package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="VuMark Detection Test", group="Test")
//@Disabled
public class BILVuMark extends BILAutonomousCommon {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();

    @Override public void runOpMode() {

        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables imageTargets = helper.loadTargets("RelicVuMark");
        VuforiaTrackable imageTemplate = imageTargets.get(0);
        imageTemplate.setName("VuMarkTemplate");

        robot.init(hardwareMap);

        waitForStart();

        imageTargets.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(imageTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }
}
