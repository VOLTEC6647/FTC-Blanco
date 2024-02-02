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
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;


@TeleOp

public class Chassis extends LinearOpMode {
    
    public static Gamepad controller1;
    public static Gamepad controller2;
    
    private Blinker control_Hub;
    private double speed=1;
    private IMU imu;
    public double baseSpeed=1;
    private Orientation orientation;
    private DcMotor intake;
    boolean hasTarget=false;
    public double gyr;

    private boolean mix = true;

    public ChassisSubsystem chassis;

    void ElevatorMethods(ElevatorSubsystem elevator){
        /*
        if(controller1.right_trigger>0.1){
            elevator.goUp(controller1.right_trigger);
        }else if(controller1.left_trigger<0.1){
            elevator.goDown(controller1.left_trigger);
        }
        */

        elevator.getPosition(telemetry);
        if(!controller2.start) {
            /*
            if (controller1.dpad_up && controller1.b) {
                elevator.elevator1.setPower(-1);
            } else if (controller1.dpad_down && controller1.b) {
                elevator.elevator1.setPower(1);
            } else if (controller1.dpad_up && controller1.a) {
                elevator.elevator1.setPower(-1);
            } else if (controller1.dpad_down && controller1.a) {
                elevator.elevator1.setPower(1);
            } else if (controller1.dpad_up) {
                elevator.elevator2.setPower(-1);
                elevator.elevator1.setPower(-1);
            } else if (controller1.dpad_down) {
                elevator.elevator2.setPower(1);
                elevator.elevator1.setPower(1);
            } else {
                elevator.elevator1.setPower(0);
                elevator.elevator2.setPower(0);
            }

             */
            if(Math.abs(controller2.right_stick_y)>0.3) {
                elevator.DebugSpeed = Math.abs(controller2.right_stick_y);
                telemetry.addData("lsthikkk",controller2.right_stick_y);
                if (controller2.right_stick_y > 0.3) {
                    elevator.goDown();
                } else if (controller2.right_stick_y < -0.3) {
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
        if(controller1.back){
            if(controller1.a){
                gyroscope.reset();
                chassis.updateTargetAngle();
            }

        }
        if(controller1.y&&false){
            chassis.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }else{
            chassis.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(controller1.start){
            if (controller1.dpad_up){
                gyroscope.setOffset(180);
                chassis.updateTargetAngle();
            }
            if (controller1.dpad_right){
                gyroscope.setOffset(270);
                chassis.updateTargetAngle();
            }
            if (controller1.dpad_down){
                gyroscope.setOffset(0);
                chassis.updateTargetAngle();
            }
            if (controller1.dpad_left){
                gyroscope.setOffset(90);
                chassis.updateTargetAngle();
            }
        }
        chassis.arcadeDrive(controller1.left_stick_x,-controller1.left_stick_y,controller1.right_stick_x,speed,gyr);

        if(controller1.right_trigger>0.1){
            speed=baseSpeed*(1.4-controller1.right_trigger);
        }else{
            speed=baseSpeed;
        }
    }
    void IMUMethods(){
        orientation=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("GyroX", AGyroscope.getX());
        //telemetry.addData("GyroY", AGyroscope.getY());
        telemetry.addData("Gyro", orientation.firstAngle);


        if(controller1.right_stick_button){
            //imu.resetYaw();

        }
    }
    /*void ArmMethods(ArmSubsystem arm, PivotSubsystem pivot){

        if(info.name=="gobilda"){
            arm.going_down =Chassis.controller2.left_stick_y < -0.2;
            arm.going_down = false;
        }

        if(!controller2.start){
            if (controller2.dpad_right) {
                if(info.name=="gobilda") {
                    arm.setPosition(0.5);
                    arm.updateArm();
                }else {
                    pivot.up();
                }

            } else if(controller2.dpad_left) {
                if(info.name=="gobilda") {
                    if(!arm.open){
                        arm.setZero();
                        arm.updateArm();
                    }
                }else {
                    pivot.down();
                }

            }else if(controller2.dpad_up){
                if(info.name=="gobilda") {
                    arm.setPosition(0.7);
                    arm.updateArm();
                }else {

                }

            }
        }

        if (controller2.right_bumper) {
            if(info.name=="rev"){
                pivot.open();
            }else{
                arm.open();
            }

        } else if(controller2.left_bumper) {
            if(info.name=="rev"){
                pivot.close();
            }else{
                arm.close();
            }
        }
        if(info.name=="gobilda") {
            if (arm.servoDelta.seconds() > 0.5) {
                arm.servoDelta.reset();
                arm.showPositions();

            }
            telemetry.addData("angle", arm.angle);
        }


    }*/

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");

        //control_Hub.setConstant(3);

        telemetry.addData("Status", "Initialized");
        /////////////////////////////
        ChassisSubsystem chassis=ChassisSubsystem.getInstance(hardwareMap,telemetry);
        ElevatorSubsystem elevator = ElevatorSubsystem.getInstance(hardwareMap,telemetry);
        //Odometry odometry = Odometry.getInstance(hardwareMap,chassis);
        GyroscopeSubsystem gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        DroneLauncherSubsystem launcher = DroneLauncherSubsystem.getInstance(hardwareMap,telemetry);
        PivotSubsystem pivot = PivotSubsystem.getInstance(hardwareMap,telemetry);

        //launcher.reset();
        telemetry.update();

        waitForStart();

        chassis.updateTargetAngle();

        //arm.setZero();

        controller1 = this.gamepad1;
        controller2 = this.gamepad2;

        if(this.gamepad2.start&this.gamepad2.back){
            controller1=this.gamepad2;
        }

        while(opModeIsActive()) {

            ChassisMethods(chassis,gyroscope);

            ElevatorMethods(elevator);

            pivot.pivotControls(this.gamepad2.a,this.gamepad2.b,this.gamepad2.dpad_right, this.gamepad2.left_bumper, this.gamepad2.right_bumper);

            telemetry.addData("posicion pinza: ", pivot.servoPivot.getPosition());
            telemetry.update();

            if (controller2.y&&!(controller2.start||controller2.back)) {
                launcher.launch();
            } else {
                launcher.reset();
            }


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