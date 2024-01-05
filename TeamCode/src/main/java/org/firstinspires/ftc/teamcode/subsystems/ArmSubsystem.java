package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {
    private static ArmSubsystem instance;
    public Servo servoL;
    public Servo servoR;
    public CRServo crservoL;
    public CRServo crservoR;

    public static ArmSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new ArmSubsystem(hardwareMap);
        }
        return instance;
    }
    public ArmSubsystem(HardwareMap hardwareMap){
        this.servoL=hardwareMap.get(Servo.class, "servoL");
        this.servoR=hardwareMap.get(Servo.class, "servoR");
        this.crservoL=hardwareMap.get(CRServo.class, "crservoL");
        this.crservoR=hardwareMap.get(CRServo.class, "crservoR");
    }

}
