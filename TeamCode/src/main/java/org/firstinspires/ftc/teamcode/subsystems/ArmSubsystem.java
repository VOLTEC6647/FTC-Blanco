package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Chassis;

public class ArmSubsystem {
    private static ArmSubsystem instance;
    public DcMotorEx armMotor;
    public CRServo crservoL;
    public CRServo crservoR;

    public Servo claw;

    private Telemetry telemetry;

    public Servo axis;

    public double NotAngle=0;
    public double NotAxis=0;

    //private boolean outside = false;

    public boolean going_down = false;

    public ElapsedTime servoDelta= new ElapsedTime();

    public boolean extended = false;

    public boolean open = false;


    public static ArmSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if (instance == null) {
            instance = new ArmSubsystem(hardwareMap, telemetry);
        }
        instance.telemetry = telemetry;

        instance.armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        instance.setZero();
        instance.updateArm();
        //instance.armMotor.
        instance.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        instance.armMotor.setPower(0.8);

        //default
        // instance.armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10,0.050003,0,0));

        //instance.armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(20,0.050003,0,0));
        instance.armMotor.setPositionPIDFCoefficients(30);

        telemetry.addData("ARMP",instance.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Threshhold",instance.armMotor.getTargetPositionTolerance());
        //default threshhold is 5
        instance.armMotor.setTargetPositionTolerance(1);

        //telemetry.addData(DcMotorEx)

        return instance;
    }
    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.armMotor=hardwareMap.get(DcMotorEx.class, "armMotor");
        this.claw=hardwareMap.get(Servo.class, "claw");
        this.axis=hardwareMap.get(Servo.class, "axis");
        this.NotAxis=0.415;
        this.updateAxis();
        //this.crservoL=hardwareMap.get(CRServo.class, "crservoL");
        //this.crservoR=hardwareMap.get(CRServo.class, "crservoR");
        //this.telemetry = telemetry;
        servoDelta.startTime();
    }

    public void showPositions(){
        telemetry.addData("armPOS", armMotor.getCurrentPosition());

    }
    public void setZero(){
        NotAngle=7;
        NotAxis=0.4;
        updateArm();

        extended = false;
    }

    public void setPosition(double position){
        if(position==1){
            NotAngle=-30;
            NotAxis=0.3;
        }
        extended = true;
        updateArm();
    }

    public void updateArm(){
        armMotor.setTargetPosition((int) NotAngle);
        if(going_down){
            //servoL.setPosition(angle-0.3);
            //servoR.setPosition(servoR.MAX_POSITION-angle+0.3);
        }else{
            //servoL.setPosition(angle);
            //servoR.setPosition(servoR.MAX_POSITION-angle);
        }

        updateAxis();
    }
    private void updateAxis(){
        if(extended){
            //outside=true;
            //axis.setPosition(angle+0.66);

        }else {

            if(!extended){
                //axis.setPosition(0);
            }
            //axis.setPosition(axis.MAX_POSITION);
        }
        axis.setPosition(NotAxis);

    }
    public void open(){
        claw.setPosition(0.55);
        open = true;

    }
    public void close(){
        claw.setPosition(0.3);
        open = false;
    }



}