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

    public ChassisSubsystem chassis;

    void ElevatorMethods(ElevatorSubsystem elevator){
        elevator.getPosition(telemetry);

        if (!controller2.start) {
            if(Math.abs(controller2.right_stick_y)>0.3) {
                elevator.DebugSpeed = Math.abs(controller2.right_stick_y);
                if (controller2.right_stick_y > 0.3) {
                    elevator.goDown();
                } else if (controller2.right_stick_y < -0.3) {
                    elevator.goUp();
                }
            } else {
                elevator.stop();
            }
        }
    }

    void ChassisMethods(ChassisSubsystem chassis, GyroscopeSubsystem gyroscope){
        gyr = gyroscope.getRotation();
        telemetry.addData("gyr",gyr);
        if(controller1.back){
            if(controller1.a){
                gyroscope.reset();
                chassis.updateTargetAngle();
            }
        }

        if (controller1.y) {
            chassis.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            chassis.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {
            chassis.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (controller1.start) {
            if (controller1.dpad_up) {
                gyroscope.setOffset(180);
                chassis.updateTargetAngle();
            }

            if (controller1.dpad_right) {
                gyroscope.setOffset(270);
                chassis.updateTargetAngle();
            }

            if (controller1.dpad_down) {
                gyroscope.setOffset(0);
                chassis.updateTargetAngle();
            }

            if (controller1.dpad_left) {
                gyroscope.setOffset(90);
                chassis.updateTargetAngle();
            }
        }

        chassis.arcadeDrive(controller1.left_stick_x,-controller1.left_stick_y,controller1.right_stick_x,speed,gyr);

        if (controller1.right_trigger>0.1) {
            speed=baseSpeed * (1.4-controller1.right_trigger);
        } else {
            speed=baseSpeed;
        }
    }

    void PivotMethods(PivotSubsystem pivot) {
        if (gamepad2.a) {
            pivot.pickup();
        } else if (gamepad2.x) {
            pivot.down();
        } else if (gamepad2.b) {
            pivot.up();
        } else if (gamepad2.left_bumper) {
            pivot.close();
        } else if (gamepad2.right_bumper) {
            pivot.open();
        }
    }

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //control_Hub.setConstant(3);
        telemetry.addData("Status", "Initialized");

        //WORKS SO FAR I THINK
        ChassisSubsystem chassis=ChassisSubsystem.getInstance(hardwareMap,telemetry);
        ElevatorSubsystem elevator = ElevatorSubsystem.getInstance(hardwareMap,telemetry);
        GyroscopeSubsystem gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        //TESTING NEEDED
        DroneLauncherSubsystem launcher = DroneLauncherSubsystem.getInstance(hardwareMap,telemetry);
        PivotSubsystem pivot = PivotSubsystem.getInstance(hardwareMap,telemetry);

        //Maybe this messes something up?? idk maybe maybe
        gyroscope.reset();
        //launcher.reset();

        waitForStart();

        controller1 = this.gamepad1;
        controller2 = this.gamepad2;

        if (this.gamepad2.start&this.gamepad2.back) {
            controller1=this.gamepad2;
        }

        while (opModeIsActive()) {

            ChassisMethods(chassis,gyroscope);

            ElevatorMethods(elevator);

//            ArmMethods(arm,pivot);

                PivotMethods(pivot);

            if (controller2.y&&!(controller2.start||controller2.back)) {
                launcher.launch();
            } else {
                launcher.reset();
            }

            //telemetry.addData("angle", arm.angle);
            telemetry.update();
        }


    }

}