package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Vision {
    private static final String VUFORIA_KEY = "AU6p3vX/////AAAAGXbGGUU5NkaliUUGq2/Kz/Z6xyNBL1fmMylvq1uMMqkFx8J4CCcF4S34r9tecpyQ0E4VJjgRklceKPzDOpiiQFxzNkZxSiIwTx4zylpK8pf6oBUg6qXji/9xlE41OyFzhKkjgCJekACepkKmviiUF94oSIRvNjOqGlVVtOKCUN3LTYF/xTIULB52NVbaXvco/9alDrM6diA/Vdfdd0qVxDnZ8mu4R7PsO19lJNcpEyNs4eSjvPlY3t75KJM1sHFFl3nUn5Piggv65VU5+sfS6VELGrGDNisMLFsp3Qk8+KSV5twxoM5cnpjGqN1CYusvkhyJAF/TBXNKb0LN0b2PNkVPIU1VYeycNax0z6HHZZ6J";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private HardwareMap hwmap;

    public Vision(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }

    public void init(String file, String... names) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwmap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hwmap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwmap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(file, names);
    }

    public void startTracking() {
        tfod.activate();
    }

    public List<Recognition> getObjects() {
        return tfod.getUpdatedRecognitions();
    }

    public void stopTracking() {
        tfod.shutdown();
    }

}
