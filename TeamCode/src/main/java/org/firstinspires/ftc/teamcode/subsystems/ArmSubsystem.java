package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Chassis;

import javax.crypto.spec.DESedeKeySpec;

public class ArmSubsystem {
    private static ArmSubsystem instance;
    public Servo servoL;
    public Servo servoR;
    public CRServo crservoL;
    public CRServo crservoR;

    public Servo claw;

    private Telemetry telemetry;

    private Servo axis;

    public double angle=0;

    private boolean outside = false;

    public boolean going_down = false;

    public ElapsedTime servoDelta= new ElapsedTime();


    public static ArmSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if (instance == null) {
            instance = new ArmSubsystem(hardwareMap, telemetry);
        }
        return instance;
    }
    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        //this.servoL=hardwareMap.get(Servo.class, "servoL");
        //this.servoR=hardwareMap.get(Servo.class, "servoR");
        this.claw=hardwareMap.get(Servo.class, "claw");
        this.axis=hardwareMap.get(Servo.class, "axis");
        //this.crservoL=hardwareMap.get(CRServo.class, "crservoL");
        //this.crservoR=hardwareMap.get(CRServo.class, "crservoR");
        this.telemetry = telemetry;
        servoDelta.startTime();
    }

    public void showPositions(){
        //telemetry.addData("servoLl", servoL.getPosition());
        //telemetry.addData("servoRr", servoR.getPosition());
        //telemetry.addData("crservoL", crservoL.getPower());
        //telemetry.addData("crservoR", crservoR.getPower());
        telemetry.addData("Axis: ", axis.getPosition());
        telemetry.addData("Claw: ", claw.getPosition());

    }
    public void setZero(){
        angle=0.07;
    }

    public void setPosition(double position){
        angle=position;
    }

    public void setAxisPosition(int desiredPosition) {
        if (desiredPosition == 1) {
            axis.setPosition(1);
        } else if (desiredPosition == 0) {
            axis.setPosition(0);
        }
    }
    public void updateArm(){
        if(going_down){
            servoL.setPosition(angle-0.3);
            servoR.setPosition(servoR.MAX_POSITION-angle+0.3);
        }else{
            servoL.setPosition(angle);
            servoR.setPosition(servoR.MAX_POSITION-angle);
        }


        updateAxis();
    }
    public void updateAxis(){
        if(angle>0.1){
            outside=true;
            axis.setPosition(angle+0.66);
        }else {
            if(!outside){
                //axis.setPosition(0);
            }
            axis.setPosition(axis.MAX_POSITION);
        }

    }
    public void open(){
        claw.setPosition(claw.MIN_POSITION);
    }
    public void close(){
        claw.setPosition(claw.MAX_POSITION);
    }



}
