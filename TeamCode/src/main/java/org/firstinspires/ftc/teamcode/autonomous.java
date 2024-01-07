package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

@Autonomous
public class autonomous extends LinearOpMode {
    private CameraSubsystem cameraSubsystem;
    @Override
    public void runOpMode() {
        cameraSubsystem = CameraSubsystem.getInstance(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
