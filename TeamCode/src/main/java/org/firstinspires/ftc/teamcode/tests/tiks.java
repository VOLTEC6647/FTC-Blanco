package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class tiks extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Statuss4dd", "Initialized");
            //telemetry 1+1
            telemetry.addData("1+1", 1+1);
            telemetry.update();
        }

    }
}
