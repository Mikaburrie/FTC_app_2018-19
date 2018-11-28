package org.firstinspires.ftc.teamcode;

/**
 * Created by mikab_000 on 10/10/2018.
 */

public class MotionProfiler {
    private double maxSpeed;
    private double acceleration;
    private double t1;
    private double t2;
    private double t3;
    private boolean running = false;

    public MotionProfiler(double maxSpeed, double acceleration) {
        this.maxSpeed = maxSpeed;
        this.acceleration = acceleration;
    }

    public String start(double distance) {

        double accelerationTime = maxSpeed / acceleration;
        double accelerationDistance = 0.5 * acceleration * Math.pow(accelerationTime, 2);

        if(accelerationDistance * 2 >= distance){
            accelerationDistance = 0.5 * distance;
            accelerationTime = Math.sqrt(2 * accelerationDistance / acceleration);
        }

        double middleDistance = distance - (2 * accelerationDistance);
        double middleTime = middleDistance / maxSpeed;
        double totalTime = middleTime + (2 * accelerationTime);

        t1 = accelerationTime;
        t2 = accelerationTime + middleTime;
        t3 = totalTime;
        running = true;
        return String.format("%f %f %f %f", accelerationDistance, middleDistance, middleTime, totalTime);
    }

    public double getSpeed(double time) {
        double vel = 0;
        if(time < t1){
            vel = acceleration * time;
        }else if(time < t2){
            vel = maxSpeed;
        }else if(time < t3){
            vel = -acceleration * (time - t3);
        }else{
            stop();
            return 0;
        }
        return vel/maxSpeed;
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        running = false;
    }
}
