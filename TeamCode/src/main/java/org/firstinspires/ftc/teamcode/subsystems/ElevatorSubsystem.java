package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.tests.Elevator2Test;

public class ElevatorSubsystem {
    private static ElevatorSubsystem instance;

    public DcMotor elevator1;
    public DcMotor elevator2;

    public double DebugSpeed =1;
    private HardwareMap hardwareMap;

    private int safezone = 100;
    private int maxzone = 10000;

    private int encoderdiff = 0;

    public Telemetry telemetry;

    public static ElevatorSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new ElevatorSubsystem(hardwareMap);

        }

        return instance;
    }
    public ElevatorSubsystem(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        this.elevator1=hardwareMap.get(DcMotor.class, "el1");
        this.elevator2=hardwareMap.get(DcMotor.class, "el2");
        this.elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        if(false){
            this.elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



    }
    public void goUp(double speed){
        if(elevator1.getCurrentPosition()<maxzone) {
            elevator1.setPower(speed*DebugSpeed);
            elevator2.setPower(speed*DebugSpeed);
        }else{
            elevator1.setPower(0.2);
            elevator2.setPower(0.2);
        }
    }
    public void goDown(double speed){
        if (elevator1.getCurrentPosition() > safezone) {
            elevator1.setPower(-speed*DebugSpeed);
            elevator2.setPower(-speed*DebugSpeed);
        } else {
            elevator1.setPower(0.2);
            elevator2.setPower(0.2);
        }

    }
    public void setPower(double speed){
        elevator1.setPower(speed*DebugSpeed);
        elevator2.setPower(speed*DebugSpeed);
    }
    public void resetEncoders(){
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void setUp(){
        elevator1.setTargetPosition(50);
    }
    public void getTarget(Telemetry telemetry){
        telemetry.addData("elevator1", elevator1.getTargetPosition());
        telemetry.addData("elevator2", elevator2.getTargetPosition());
    }


}
