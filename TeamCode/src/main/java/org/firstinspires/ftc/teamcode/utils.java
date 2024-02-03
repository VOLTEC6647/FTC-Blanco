package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class utils {
    public static int shortestPathDirection(int angleA, int angleB) {
        int difference = Math.abs(angleA - angleB);
        int shortestPath = Math.min(difference, 360 - difference);

        // Determine direction
        if ((angleB - angleA + 360) % 360 == shortestPath) {
            return 1;
        } else {
            return -1;
        }
    }
    public static int calculateRotation(int angleA, int angleB) {
        int forwardRotation = (angleB - angleA + 360) % 360;
        int backwardRotation = (angleA - angleB + 360) % 360;

        // Determine if forward rotation is shorter, otherwise use backward
        if (forwardRotation <= backwardRotation) {
            return forwardRotation;
        } else {
            return -backwardRotation;
        }
    }

    private static ElapsedTime time = new ElapsedTime();
    public static void waitMs(int ms, Telemetry telemetry){
        time.reset();
        time.startTime();
        while (time.milliseconds()<ms){
            //telemetry.addData("state","waiting "+ms);
            //telemetry.addData("time",ms);
            //telemetry.update();
        }
    }
    public static void intakeTime(int ms, Telemetry telemetry, DcMotor intake,double power){
        time.reset();
        time.startTime();
        while (time.milliseconds()<ms){
            intake.setPower(power);
            telemetry.addData("time",ms);
            telemetry.update();
        }
    }
}
