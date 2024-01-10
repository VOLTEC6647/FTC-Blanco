package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
public class GripperTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        CRServo axis = hardwareMap.get(CRServo.class, "axis");
        CRServo caxis = hardwareMap.get(CRServo.class, "caxis");
        caxis.setDirection(DcMotorSimple.Direction.FORWARD);
        //ArmSubsystem armSubsystem = ArmSubsystem.getInstance(hardwareMap,telemetry);
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.right_trigger>0.1){
                axis.setPower(gamepad1.right_trigger);
                caxis.setPower(gamepad1.right_trigger);
            }else if(gamepad1.left_trigger>0.1){
                axis.setPower(-gamepad1.left_trigger);
                caxis.setPower(-gamepad1.left_trigger);
            }else {
                axis.setPower(0);
                caxis.setPower(0);
            }
        }

    }
}
