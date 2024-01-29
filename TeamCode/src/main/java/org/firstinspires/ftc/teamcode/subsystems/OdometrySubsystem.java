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

    private GyroscopeSubsystem gyr;



    public OdometrySubsystem(HardwareMap hardwareMap, ChassisSubsystem chassis,GyroscopeSubsystem gyr, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        this.yEncoder = hardwareMap.get(DcMotor.class, "BR");
        this.chassis = chassis;
        this.telemetry = telemetry;
        this.gyr = gyr;
    }
    public static OdometrySubsystem getInstance(HardwareMap hardwareMap, ChassisSubsystem chassis, GyroscopeSubsystem gyr, Telemetry telemetry){
        if (instance == null) {
            instance = new OdometrySubsystem(hardwareMap, chassis, gyr, telemetry);
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

    public void goToYolo(int x, int y, double speed) {
        resetEncoders();
        while (Math.abs(y - getYDist())+Math.abs(x - getXDist()) > 5) {
            int xdir=0;
            int ydir=0;
            if(x!=0){
                if (x - getXDist()>0){
                    xdir=-1;
                }else if (x - getXDist()<0){
                    xdir=1;
                }
            }

            if(y!=0) {
                if (y - getYDist() > 0) {
                    ydir = -1;
                } else if (y - getYDist() < 0) {
                    ydir = 1;
                }
            }
            //chassis.arcadeDrive(,-,0,speed,gyr.getRotation());;
            telemetry.addData("gyr",gyr.getRotation());
            chassis.arcadeDrive(xdir,ydir,0,speed,gyr.getRotation());;
            telemetry.addData("diffx",Math.abs(x - getXDist()));
            telemetry.addData("diffy",Math.abs(y - getYDist()));
            telemetry.update();
        }
        chassis.setMotors(0,0,0,0);
    }
    public void rotatePeroMejor(int dir) {

        while (Math.abs(gyr.getRotation()-dir)>5) {
            chassis.arcadeDrive(0,0,0,0.7,gyr.getRotation());;
            telemetry.update();
        }
    }


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