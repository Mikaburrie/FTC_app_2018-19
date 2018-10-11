package org.firstinspires.ftc.teamcode;

/**
 * Created by mikab_000 on 10/10/2018.
 */

public class MotionProfiler {
    public double maxSpeed;
    public double acceleration;
    private double accelerationTime;
    private double accelerationDistance;
    private double middleDistance;
    private double middleTime;
    private double totalTime;
    private double t0;
    private double t1;
    private double t2;
    private double t3;

    public MotionProfiler(double maxSpeed, double acceleration) {
        this.maxSpeed = maxSpeed;
        this.acceleration = acceleration;
    }

    public void setDistance(double distance) {
        accelerationTime = maxSpeed / acceleration;
        accelerationDistance = 1/2 * acceleration * Math.pow(accelerationTime, 2);

        if(accelerationDistance * 2 >= distance){
            accelerationDistance = 1/2 * distance;
            accelerationTime = Math.sqrt(2 * accelerationDistance / acceleration);
        }

        middleDistance = distance - (2 * accelerationDistance);
        middleTime = middleDistance / maxSpeed;
        totalTime = middleTime + (2 * accelerationTime);

        t0 = 0;
        t1 = accelerationTime;
        t2 = accelerationTime + middleTime;
        t3 = totalTime;
    }

    public void start() {

    }

    public double getSpeed(double time) {
        double vel = 0;
        if(time < t1){
            vel = acceleration * time;
        }else if(time < t2){
            vel = maxSpeed;
        }else if(time < t3){
            vel = -acceleration * (time - t3);
        }
        return vel;
    }

    public void stop() {

    }
}
