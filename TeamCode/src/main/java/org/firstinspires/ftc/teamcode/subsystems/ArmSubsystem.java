package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {
    private static ArmSubsystem instance;
    private Servo servo;
    private CRServo crservo;
    public ArmSubsystem(HardwareMap hardwareMap){
        this.servo = hardwareMap.get(Servo.class, "servo");
        this.crservo = hardwareMap.get(CRServo.class, "servo");
    }
    public static ArmSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new ArmSubsystem(hardwareMap);
        }
        return instance;
    }
    public void setPosition(double position){
        servo.setPosition(position);
    }
    public void setPower(double power){
        crservo.setPower(power);
    }



}