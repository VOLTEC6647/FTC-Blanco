package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;

@Autonomous
public class autonomous extends LinearOpMode {
    private OpenCVSubsystem openCVSubsystem;
    @Override
    public void runOpMode() {
        openCVSubsystem = OpenCVSubsystem.getInstance(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
