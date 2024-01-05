package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;


@TeleOp

public class EncoderTest extends LinearOpMode {

    private Blinker control_Hub;
    private double speed = 1;
    private IMU imu;
    private double baseSpeed = 0.5;
    private Orientation orientation;
    private DcMotor xEncoder;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(IMU.class, "imu");
        Odometry odometry = Odometry.getInstance(hardwareMap);
        waitForStart();



        while(opModeIsActive()) {
            odometry.update();

            telemetry.addData("X", odometry.position().x);
            telemetry.update();

        }
    }

}