package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;

import java.io.File;

@Autonomous
public class autonomousRevisit extends LinearOpMode {
    private OpenCVSubsystem camera;
    private ChassisSubsystem chassis;
    private GyroscopeSubsystem gyroscope;
    private ElevatorSubsystem elevator;
    private OdometrySubsystem odometry;
    private ArmSubsystem arm;
    private Blinker control_Hub;
    private double speed = 1;
    private double baseSpeed = 0.5;
    private DcMotor intake;
    boolean hasTarget = false;

    private String prop="";
    private String team="";
    private String auto="";
    private String robotPosition ="";
    public double gyr;

//    public void rotateAngle(double target) {
//        double difference = Math.abs(target - gyroscope.getRotation());
//        while (difference > 2) {
//            chassis.targetAngle = target;
//            chassis.
//            difference = Math.abs(target - gyroscope.getRotation());
//        }
//        odometry.resetEncoders();
//    }
    @Override
    public void runOpMode() {

        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        intake = hardwareMap.get(DcMotor.class, "intake");

        camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        chassis = ChassisSubsystem.getInstance(hardwareMap,telemetry);
        gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        elevator = ElevatorSubsystem.getInstance(hardwareMap,telemetry);
        odometry = OdometrySubsystem.getInstance(hardwareMap, chassis,gyroscope, telemetry);
        arm = ArmSubsystem.getInstance(hardwareMap, telemetry);

        chassis.autonomous=true;

        odometry.resetEncoders();

        gyroscope.reset();

        boolean rotation = true;
        boolean test = true;

        chassis.REnabled=true;

        loadparameters();
        telemetry.update();

        //intake.setPower(0.3);
        //utils.waitMs(2000, telemetry);
        //intake.setPower(0);
        waitForStart();

        //arm.open();


        if(team!=""&&robotPosition!=""&&prop!=""&&auto!="disabled") {
            /*
            elevator.DebugSpeed=0.6;
            elevator.goUp();
            odometry.goToYolo(10,10,0.5);
            elevator.stop();
            arm.setPosition(0.3);
            arm.updateArm();
            */

            //point to prop
            odometry.goToYolo(-20,0,0.5,false);
            odometry.rotatePeroMejor(90);
            odometry.rotatePeroMejor(180);
            //intake.setPower(0.05);
            odometry.goToYolo(-45,0,0.5,false);
            //intake.setPower(0.05);
            odometry.goToYolo(0,7,0.5,false);
            //intake.setPower(0.1);
            //utils.waitMs(500, telemetry);
            utils.intakeTime(1000,telemetry,intake,0.5);
            intake.setPower(0);
            odometry.rotatePeroMejor(0);



        }else{
            arm.close();
            //odometry.goToYolo(0,-64,0.5);

            //odometry.goToYolo(-125,0,0.5);
            //odometry.goToYolo(-222,0,0.5);

            //chassis.targetAngle+=;
            //odometry.goToYolo(0,-20,0.3);
        }
        chassis.setMotors(0,0,0,0);
        while (opModeIsActive()){
            String amogus="yes";
        }

    }
    private void loadparameters(){
        File myFileName;

        myFileName = AppUtil.getInstance().getSettingsFile("prop.txt");
        prop = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        team = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("auto.txt");
        auto = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("position.txt");
        robotPosition = ReadWriteFile.readFile(myFileName);

        telemetry.addData("Prop", prop);
        telemetry.addData("Team", team);
        telemetry.addData("Auto", auto);
        telemetry.addData("Position", robotPosition);

    }
}
