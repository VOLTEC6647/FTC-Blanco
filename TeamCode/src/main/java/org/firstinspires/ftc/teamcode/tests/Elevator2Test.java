package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

@TeleOp

public class Elevator2Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        ElevatorSubsystem elevator = ElevatorSubsystem.getInstance(hardwareMap);

        telemetry.update();


        waitForStart();



        while(opModeIsActive()) {
            elevator.resetEncoders();
            elevator.getTarget(telemetry);
            elevator.setUp();
            telemetry.update();
        }

    }
}