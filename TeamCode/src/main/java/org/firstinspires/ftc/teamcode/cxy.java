package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystemmm;

@Autonomous
public class cxy extends LinearOpMode {
    private OpenCVSubsystem camera;

    @Override
    public void runOpMode() {



        camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);

        boolean rotation = true;
        boolean test = true;

        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("ccx",camera.getcx());
            telemetry.addData("ccy",camera.getcy());
            //telemetry.addData("area",camera.getArea());
            telemetry.update();
        }

    }

}
