package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Chassis;

public class ArmSubsystem {
    private static ArmSubsystem instance;
    public Servo servoL;
    public Servo servoR;
    public CRServo crservoL;
    public CRServo crservoR;

    private Telemetry telemetry;

    public static ArmSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if (instance == null) {
            instance = new ArmSubsystem(hardwareMap, telemetry);
        }
        return instance;
    }
    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.servoL=hardwareMap.get(Servo.class, "servoL");
        this.servoR=hardwareMap.get(Servo.class, "servoR");
        //this.crservoL=hardwareMap.get(CRServo.class, "crservoL");
        //this.crservoR=hardwareMap.get(CRServo.class, "crservoR");
        this.telemetry = telemetry;
    }

    public void showPositions(){
        telemetry.addData("servoL", servoL.getPosition());
        telemetry.addData("servoR", servoR.getPosition());
        //telemetry.addData("crservoL", crservoL.getPower());
        //telemetry.addData("crservoR", crservoR.getPower());

    }
    public void setZero(){
        servoL.setPosition(0);
        servoR.setPosition(0);
    }



}
