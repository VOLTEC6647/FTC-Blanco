package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;


@TeleOp

public class Chassis extends LinearOpMode {

    private Blinker control_Hub;
    private double speed=1;
    private IMU imu;
    private double baseSpeed=0.5;
    private Orientation orientation;
    private DcMotor intake;
    boolean hasTarget=false;
    public double gyr;
    public Gamepad controller1;
    public Gamepad controller2;


    public ChassisSubsystem chassis;

    void ElevatorMethods(ElevatorSubsystem elevator){
        /*
        if(this.gamepad1.right_trigger>0.1){
            elevator.goUp(this.gamepad1.right_trigger);
        }else if(this.gamepad1.left_trigger<0.1){
            elevator.goDown(this.gamepad1.left_trigger);
        }
        */

        if(!this.gamepad1.start) {
            /*if (this.gamepad1.dpad_up && this.gamepad1.b) {
                elevator.elevator1.setPower(-1);
            } else if (this.gamepad1.dpad_down && this.gamepad1.b) {
                elevator.elevator1.setPower(1);
            } else if (this.gamepad1.dpad_up && this.gamepad1.a) {
                elevator.elevator1.setPower(-1);
            } else if (this.gamepad1.dpad_down && this.gamepad1.a) {
                elevator.elevator1.setPower(1);
            } else if (this.gamepad1.dpad_up) {
                elevator.elevator2.setPower(-1);
                elevator.elevator1.setPower(-1);
            } else if (this.gamepad1.dpad_down) {
                elevator.elevator2.setPower(1);
                elevator.elevator1.setPower(1);
            } else {
                elevator.elevator1.setPower(0);
                elevator.elevator2.setPower(0);
            }*/
            if(Math.abs(gamepad2.right_stick_y)>0.3) {
                elevator.DebugSpeed = Math.abs(gamepad2.right_stick_y);
                if (gamepad2.right_stick_y > 0.3) {
                    elevator.goDown();
                } else if (gamepad2.right_stick_y < -0.3) {
                    elevator.goUp();
                }
            }else {
                elevator.stop();
            }
        }

    }
    void ChassisMethods(ChassisSubsystem chassis, GyroscopeSubsystem gyroscope){
        //IMUMethods();
        gyr = gyroscope.getRotation();
        //gyr=0;
        telemetry.addData("gyr",gyr);
        if(this.gamepad1.back){
            if(this.gamepad1.a){
                gyroscope.reset();
            }

        }
        if(this.gamepad1.x){
            chassis.FrontLeftMotor.setPower(1);
        }else if(this.gamepad1.y){
            chassis.FrontLeftMotor.setPower(-1);
        }else{
            chassis.FrontLeftMotor.setPower(0);
        }
        if(this.gamepad1.start){
            if (this.gamepad1.dpad_up){
                gyroscope.setOffset(180);
                chassis.updateTargetAngle();
            }
            if (this.gamepad1.dpad_right){
                gyroscope.setOffset(270);
                chassis.updateTargetAngle();
            }
            if (this.gamepad1.dpad_down){
                gyroscope.setOffset(0);
                chassis.updateTargetAngle();
            }
            if (this.gamepad1.dpad_left){
                gyroscope.setOffset(90);
                chassis.updateTargetAngle();
            }
        }
        chassis.arcadeDrive(this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y,this.gamepad1.right_stick_x,speed,gyr);

        if(this.gamepad1.right_bumper){
            speed=baseSpeed*0.5;
        }else{
            speed=baseSpeed;
        }


    }
    void IMUMethods(){
        orientation=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("GyroX", AGyroscope.getX());
        //telemetry.addData("GyroY", AGyroscope.getY());
        telemetry.addData("Gyro", orientation.firstAngle);

        if(this.gamepad1.a){
            //intake.setPower(-1);

        }else{
            //intake.setPower(0);
        }
        if(this.gamepad1.right_stick_button){
            //imu.resetYaw();

        }
    }
    void ArmMethods(){

    }


    @Override
    public void runOpMode() {
        //this.telemetry = telemetry;
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(IMU.class, "imu");
        if(Parameters.robot=="marvin"){
            intake = hardwareMap.get(DcMotor.class, "intake");
        }


        telemetry.addData("Status", "Initialized");
        /////////////////////////////
        ChassisSubsystem chassis=ChassisSubsystem.getInstance(hardwareMap,telemetry);
        ElevatorSubsystem elevator = ElevatorSubsystem.getInstance(hardwareMap);
        //Odometry odometry = Odometry.getInstance(hardwareMap,chassis);
        //ArmSubsystem arm = ArmSubsystem.getInstance(hardwareMap, telemetry);
        GyroscopeSubsystem gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        PivotSubsystem pivot = PivotSubsystem.getInstance(hardwareMap, telemetry);/////////////////////////////

        //orientation=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //GyroscopeSubsystem gyroscope=GyroscopeSubsystem.getInstance();

        //telemetry.addData("GyroX", orientation.secondAngle);
        //telemetry.addData("GyroY", AndroidOrientation().getAngle().secondPosition());
        telemetry.update();


        waitForStart();

        controller1 = this.gamepad1;
        controller2 = this.gamepad2;

        if(this.gamepad2.start&this.gamepad2.back){
            controller1=this.gamepad2;
        }
        //arm.setZero();

        while(opModeIsActive()) {

            ChassisMethods(chassis,gyroscope);

            ElevatorMethods(elevator);

            ArmMethods();

            elevator.goDown();

            pivot.pivotControls(this.gamepad2.a,this.gamepad2.b,this.gamepad2.left_bumper, this.gamepad2.right_bumper);

            telemetry.update();

        }
        /*
        odometry.update();
        telemetry.addData("X", odometry.position().x);

        if(hasTarget){
            odometry.moveToCoords(2000,3000);

        }
        */

    }

}