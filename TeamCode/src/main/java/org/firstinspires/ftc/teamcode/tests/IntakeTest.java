package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.get(DcMotor.class, "BR");//
        waitForStart();
        while(opModeIsActive()) {
            if(this.gamepad1.x){
                intake.setPower(1);
            }else if(this.gamepad1.y){
                intake.setPower(-1);
            }else{
                intake.setPower(0);
            }
        }
    }
}