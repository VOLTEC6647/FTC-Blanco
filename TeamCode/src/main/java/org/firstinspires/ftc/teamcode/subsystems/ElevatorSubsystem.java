package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem {
    private static ElevatorSubsystem instance;

    public DcMotor elevator1;
    public DcMotor elevator2;

    public DcMotor elevator1enc;
    public DcMotor elevator2enc;

    public double DebugSpeed =1;
    private HardwareMap hardwareMap;

    private int minzone = 300;
    private int maxzone = 1980;

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
        this.elevator1=hardwareMap.get(DcMotor.class, "elL");
        this.elevator2=hardwareMap.get(DcMotor.class, "elR");

        this.elevator1enc=hardwareMap.get(DcMotor.class, "elL");
        this.elevator2enc=hardwareMap.get(DcMotor.class, "elR");

        this.elevator1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.elevator2.setDirection(DcMotorSimple.Direction.FORWARD);
        if(false){
            this.elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void goDown(){
        if(true||-elevator1.getCurrentPosition() > minzone) {
            elevator1.setPower(DebugSpeed);
            elevator2.setPower(-DebugSpeed);
        }else{
            elevator1.setPower(0.2);
            elevator2.setPower(-0.2);
        }
    }
    public void goUp(){
        if (true||-elevator1.getCurrentPosition()<maxzone) {
            elevator1.setPower(-DebugSpeed);
            elevator2.setPower(DebugSpeed);
        } else {
            elevator1.setPower(-0.2);
            elevator2.setPower(0.2);
        }

    }
    public void stop(){
        elevator1.setPower(0);
        elevator2.setPower(0);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if((-elevator1.getCurrentPosition()<0||elevator2.getCurrentPosition()<0)){
            resetEncoders();
        }
    }
    public void setPower(double speed){
        elevator1.setPower(speed*DebugSpeed);
        elevator2.setPower(speed*DebugSpeed);
    }
    public void resetEncoders(){
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setUp(){
        elevator1.setTargetPosition(50);
    }
    public void getTarget(Telemetry telemetry){
        telemetry.addData("elevator1", elevator1.getTargetPosition());
        telemetry.addData("elevator2", elevator2.getTargetPosition());
    }
    public void getPosition(Telemetry telemetry){
        telemetry.addData("elevator1", -elevator1.getCurrentPosition());
        telemetry.addData("elevator2", elevator2.getCurrentPosition());
    }


}
