package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


@TeleOp

public class ElevatorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        ElevatorSubsystem elevator = ElevatorSubsystem.getInstance(hardwareMap);

        telemetry.update();

        waitForStart();



        while(opModeIsActive()) {

            telemetry.addData("elevator1",elevator.elevator1.getCurrentPosition());
            if(this.gamepad1.dpad_up&&this.gamepad1.b){
                elevator.elevator1.setPower(-1);
            }else if(this.gamepad1.dpad_down&&this.gamepad1.b){
                elevator.elevator1.setPower(1);
            }else if(this.gamepad1.dpad_up&&this.gamepad1.a){
                elevator.elevator1.setPower(-1);
            }else if(this.gamepad1.dpad_down&&this.gamepad1.a){
                elevator.elevator1.setPower(1);
            }else if(this.gamepad1.dpad_up){
                elevator.elevator2.setPower(-1);
                elevator.elevator1.setPower(-1);
            }else if(this.gamepad1.dpad_down){
                elevator.elevator2.setPower(1);
                elevator.elevator1.setPower(1);
            }else{
                elevator.elevator1.setPower(0);
                elevator.elevator2.setPower(0);
            }
        }

    }
}