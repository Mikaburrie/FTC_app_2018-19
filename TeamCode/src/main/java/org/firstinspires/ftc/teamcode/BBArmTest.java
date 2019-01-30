package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Armtest", group="BIL")
public class BBArmTest extends TeleopCommon{

    BBArmTest(){}

    //@Override
    ArmControl arm;
    public void init() {
        //Initializes all robot hardware parts
        robot.init(hardwareMap);

        arm = new ArmControl(robot.switchArm);
    }

    @Override
    public void loop() {
        robot.motorArm.setPower(arm.getMotorSpeed(gamepad1.dpad_left,gamepad1.dpad_right,robot.motorArm.getCurrentPosition()));
        telemetry.addData("Arm speed", String.format("%f", arm.getMotorSpeed(gamepad1.dpad_left,gamepad1.dpad_right,robot.motorArm.getCurrentPosition())));
        telemetry.addData("Arm angle", String.format("%f", arm.encValueToDegrees(robot.motorArm.getCurrentPosition())));

    }
    @Override
    public void stop() {

    }
}
