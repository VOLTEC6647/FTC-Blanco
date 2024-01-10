package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncherSubsystem;

@TeleOp
public class LauncherTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DroneLauncherSubsystem launcher = DroneLauncherSubsystem.getInstance(hardwareMap, telemetry);
        waitForStart();
        launcher.launch();


        while (opModeIsActive()){
            telemetry.addData("active",1);
        }
    }
}
