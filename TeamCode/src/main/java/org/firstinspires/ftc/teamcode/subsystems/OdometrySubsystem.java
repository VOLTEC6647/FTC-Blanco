package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.BreakIterator;
import java.time.Year;

public class OdometrySubsystem {
    public int fieldX;
    public int fieldY;

    public int robotX=0;
    public int robotY=0;

    private HardwareMap hardwareMap;

    public DcMotor xEncoder;
    public DcMotor yEncoder;
    private Telemetry telemetry;
    private ChassisSubsystem chassis;
    private static OdometrySubsystem instance;

    public OdometrySubsystem(HardwareMap hardwareMap, ChassisSubsystem chassis, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        this.yEncoder = hardwareMap.get(DcMotor.class, "FL");
        this.chassis = chassis;
        this.telemetry = telemetry;
    }
    public static OdometrySubsystem getInstance(HardwareMap hardwareMap, ChassisSubsystem chassis, Telemetry telemetry){
        if (instance == null) {
            instance = new OdometrySubsystem(hardwareMap, chassis, telemetry);
        }
        return instance;
    }

    public class coordinates{
        public int x;
        public int y;
        public coordinates(int x, int y){
            this.x=x;
            this.y=y;
        }
    }
    public coordinates position(){
        return new coordinates(fieldX,fieldY);
    }
    public void update(){
        fieldX = robotX + xEncoder.getCurrentPosition();
        fieldY = robotY + yEncoder.getCurrentPosition();
    }
    public void resetEncoders(){
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    final double WHEELCIRCUMFERENCE = 4.8 * Math.PI;
    final int TICKS = 2000;
    public double getXDist() {
        return (xEncoder.getCurrentPosition() * WHEELCIRCUMFERENCE) / TICKS;
    }
    public double getYDist() {
        return (yEncoder.getCurrentPosition() * WHEELCIRCUMFERENCE) / TICKS;
    }
    public void printEncoders() {
        telemetry.addData("xEncoder", xEncoder.getCurrentPosition());
        telemetry.addData("yEncoder", yEncoder.getCurrentPosition());
        telemetry.addData("X: ", getXDist());
        telemetry.addData("Y: ", getYDist());
    }
    final double KP = 0.05;

    public void goTo(int x, int y) {
//        resetEncoders();

        double yError=0;
        double xError=0;

        while (Math.abs(yError)+Math.abs(xError) > 10) {
            yError = y - getYDist() * -1;
            xError = x - getXDist() * 1;
            double ySpeed = yError * KP;
            double xSpeed = xError * KP;
            telemetry.addData("yError", yError);
            telemetry.addData("xError: ", xError);
            telemetry.addData("poss", getYDist());

            //chassis.arcadeDrive(xSpeed,ySpeed,0,1,gyr.getRotation());
            chassis.moveY(ySpeed);
            chassis.moveX(xSpeed);

            telemetry.update();

        }


    }
    public boolean rotateTo(double degree, double target) {
        final double KP2 = 0.018;
        double rError = target - degree;
        double rSpeed = rError * KP2;

        chassis.moveR(rSpeed);

        return (Math.abs(rError) > 2);
    }

}