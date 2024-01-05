package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;

@TeleOp

public class GyroscopeTest extends LinearOpMode {

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            GyroscopeSubsystem gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);

            waitForStart();
            while(opModeIsActive()) {
                telemetry.addData("Rotation", gyroscope.getRotation());
                telemetry.update();
            }
        }

}