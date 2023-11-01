package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PositionTrackerSubsystem;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;



@TeleOp

public class Chassis extends LinearOpMode {

    private Blinker control_Hub;
    private double speed=1;
    private IMU imu;
    private double baseSpeed=0.5;
    private Orientation orientation;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");
        if(Parameters.robot=="marvin"){
            intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ChassisSubsystem chassis=ChassisSubsystem.getInstance(hardwareMap,telemetry);
        PositionTrackerSubsystem positionTracker=PositionTrackerSubsystem.getInstance(hardwareMap);

        orientation=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //GyroscopeSubsystem gyroscope=GyroscopeSubsystem.getInstance();

        telemetry.addData("GyroX", orientation.secondAngle);
        //telemetry.addData("GyroY", AndroidOrientation().getAngle().secondPosition());
        telemetry.update();

        waitForStart();



        while(opModeIsActive()) {
            orientation=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            chassis.arcadeDrive(this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y,this.gamepad1.right_stick_x,speed,-orientation.firstAngle);
            if(this.gamepad1.right_bumper){
                speed=baseSpeed*0.5;
            }else{
                speed=baseSpeed;
            }
            //telemetry.addData("GyroX", AGyroscope.getX());
            //telemetry.addData("GyroY", AGyroscope.getY());
            telemetry.addData("Gyro", orientation.firstAngle);
            telemetry.update();

            if(this.gamepad1.a){
                //intake.setPower(-1);

            }else{
                //intake.setPower(0);
            }
            if(this.gamepad1.right_stick_button){
                imu.resetYaw();

            }

        }
    }

}