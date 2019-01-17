package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DirectionHandler {
    public BNO055IMU imu;
    double targetDirection;
    double currentDirection;
    Orientation angles;

    public DirectionHandler(BNO055IMU unit) {
        imu = unit;
    }

    public void setTargetDirection(double angle) {
        targetDirection = angle;
    }

    public void setTargetDirection() {
        setTargetDirection(getDirection());
    }

    public double getDirection() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentDirection = (double)angles.firstAngle;
        return currentDirection;
    }

    public double driveAtTargetDirection(double speed) {
        double diff = (targetDirection - getDirection()) % 360.0;
        if(Math.abs(diff) > 180){
            diff = (360 - Math.abs(diff)) * (diff < 0 ? -1 : 1);
        }
        return diff/90.0*speed;
    }
}
