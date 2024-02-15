package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.subsystems.ODSub.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ODSub;
import org.firstinspires.ftc.teamcode.subsystems.ObjectDetectionNOTSubsystem;

@TeleOp

public class objectTest extends LinearOpMode {

    int cycles = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        ODSub sub = new ODSub();

        sub.telemetry=telemetry;
        sub.hardwareMap = hardwareMap;


        //initTfod(telemetry,hardwareMap,"blue");

        waitForStart();

        sub.initTfod();

        //sub.visionPortal.setProcessorEnabled(sub.tfod, false);

        sleep(10000);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                cycles++;
                telemetry.addData("cycles",cycles);
                sub.telemetryTfod();
                //sub.visionPortal
                telemetry.update();
                sleep(1000);
                if(sub.foundprop){
                    break;
                }
            }
        }
        sub.visionPortal.close();
        sleep(5000);


    }
}