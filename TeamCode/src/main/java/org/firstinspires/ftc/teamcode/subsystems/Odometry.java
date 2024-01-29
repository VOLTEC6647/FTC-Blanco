package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    public int fieldX;
    public int fieldY;

    public int robotX=0;
    public int robotY=0;

    private HardwareMap hardwareMap;

    private DcMotor xEncoder;
    private DcMotor yEncoder;

    private ChassisSubsystem chassis;

    private static Odometry instance;
    public Odometry(HardwareMap hardwareMap, ChassisSubsystem chassis){
        this.hardwareMap=hardwareMap;
        this.xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        this.yEncoder = hardwareMap.get(DcMotor.class, "yEncoder");
        this.chassis = chassis;
    }
    public static Odometry getInstance(HardwareMap hardwareMap, ChassisSubsystem chassis){
        if (instance == null) {
            instance = new Odometry(hardwareMap, chassis);
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
    public void moveToCoords(int x, int y){
        int xDistance = x - fieldX;
        int yDistance = y - fieldY;

        int maxpower = 1;
        int minpower = 0;
        int xpower = 0;
        int ypower = 0;
        int breakDistance = 10000;


        if(xDistance>breakDistance){
            xpower = maxpower;
        }
        else if(xDistance<-breakDistance){
            xpower = -maxpower;
        }else if (xDistance>0){
            xpower = maxpower*(xDistance/breakDistance);
        }else if (xDistance<0){
            xpower = -maxpower*(xDistance/breakDistance);
        }

        if(yDistance>breakDistance){
            ypower = maxpower;
        }
        else if(yDistance<-breakDistance){
            ypower = -maxpower;
        } else if (yDistance>0){
            ypower = maxpower*(yDistance/breakDistance);
        }else if (yDistance<0){
            ypower = -maxpower*(yDistance/breakDistance);
        }
        chassis.arcadeDrive(xpower,ypower,0,0.4,0);

    }






}