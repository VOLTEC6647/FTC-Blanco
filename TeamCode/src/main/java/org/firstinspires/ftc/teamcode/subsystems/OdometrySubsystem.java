package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Types;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils;

import java.text.BreakIterator;
import java.time.Year;
import java.util.Optional;

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
        this.xEncoder = hardwareMap.get(DcMotor.class, "BR");
        this.yEncoder = hardwareMap.get(DcMotor.class, "FR");
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
        return (-xEncoder.getCurrentPosition() * WHEELCIRCUMFERENCE*encoderOrientationX) / TICKS;
    }
    public double getYDist() {
        return (-yEncoder.getCurrentPosition() * WHEELCIRCUMFERENCE*encoderOrientationY) / TICKS;
    }
    public void printEncoders() {
        telemetry.addData("xEncoder", xEncoder.getCurrentPosition());
        telemetry.addData("yEncoder", yEncoder.getCurrentPosition());
        telemetry.addData("X: ", getXDist());
        telemetry.addData("Y: ", getYDist());
    }
    final double KP = 0.05;

    public void goToYolo(int x, int y, double speed, Boolean persist) {
        Telemetry.Item status=null;
        time.reset();
        time.startTime();
        resetEncoders();
        while (Math.abs(y - getYDist())+Math.abs(x - getXDist()) > 8&&time.milliseconds()<timeout) {
            status = telemetry.addData("state","moving "+x+"-"+y);
            int xdir=0;
            int ydir=0;
            int xOffset = 1;
            int yOffset = 1;
            if(x!=0){
                if (x - getXDist()>4){
                    xdir=1;
                }else if (x - getXDist()<-4){
                    xdir=-1;
                }
            }

            if(y!=0) {
                if (y - getYDist() > 4) {
                    ydir = 1;
                } else if (y - getYDist() < -4) {
                    ydir = -1;
                }
            }
            double multiplier = 1;
            if(Math.abs(y - getYDist())+Math.abs(x - getXDist()) < 20){
                speed=0.4;
            }
            //chassis.arcadeDrive(,-,0,speed,gyr.getRotation());;
            telemetry.addData("diffx",Math.abs(x - getXDist()));
            telemetry.addData("diffy",Math.abs(y - getYDist()));
            telemetry.addData("gyr",gyr.getRotation());
            chassis.arcadeDrive(xdir,ydir,0,speed,gyr.getRotation());
            telemetry.update();
        }
        if(!persist){
            chassis.setMotors(0,0,0,0);
        }
        telemetry.removeItem(status);

    }

    public int timeout=5000;
    private int encoderOrientationX = 1;
    private int encoderOrientationY = 1;

    private static ElapsedTime time = new ElapsedTime();

    public void rotatePeroMejor(int dir) {
        Telemetry.Item status=null;
        time.reset();
        time.startTime();
        chassis.targetAngle=dir;
        while (Math.abs(gyr.getRotation()-dir)>4&&time.milliseconds()<timeout) {
            while (Math.abs(gyr.getRotation() - dir) > 4&&time.milliseconds()<timeout) {
                status = telemetry.addData("state", "rotating "+dir);
                telemetry.addData("timeout",timeout-time.milliseconds());

                if(Math.abs(gyr.getRotation() - dir) < 20){
                    chassis.arcadeDrive(0, 0, 0, 0.3, gyr.getRotation());
                }else{
                    chassis.arcadeDrive(0, 0, 0, 0.7, gyr.getRotation());
                }

                telemetry.update();

                if(!gyr.navx.isConnected()){
                    throw new IllegalArgumentException("navx unalive");
                }
            }
            utils.waitMs(20,telemetry);
        }
        telemetry.removeItem(status);
        //recuerdenme cambiar esto ántes del nacional
        if(dir==0){
            encoderOrientationX=1;
            encoderOrientationY=1;
        }
        if(dir==180){
            encoderOrientationX=-1;
            encoderOrientationY=-1;
        }
    }

    public void rotatePeroExacto(int dir) {
        rotatePeroMejor(dir);
        /*
        time.reset();
        time.startTime();
        chassis.targetAngle=dir;
        while (Math.abs(gyr.getRotation()-dir)>5&&time.milliseconds()<timeout) {
            while (Math.abs(gyr.getRotation() - dir) > 5&&time.milliseconds()<timeout) {
                telemetry.addData("state", "rotating exact");
                telemetry.addData("timeout",timeout-time.milliseconds());
                chassis.arcadeDrive(0, 0, 0, 0.3, gyr.getRotation());
                ;
                telemetry.update();
            }
            utils.waitMs(50,telemetry);
        }
        //recuerdenme cambiar esto ántes del nacional
        if(dir==0){
            encoderOrientationX=1;
            encoderOrientationY=1;
        }
        if(dir==180){
            encoderOrientationX=-1;
            encoderOrientationY=-1;
        }
        */
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