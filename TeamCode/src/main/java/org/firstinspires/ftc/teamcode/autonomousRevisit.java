package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystemmm;

import java.io.File;
import java.util.Objects;

//@Autonomous
public class autonomousRevisit extends LinearOpMode {
    private OpenCVSubsystemmm camera;
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

        camera = OpenCVSubsystemmm.getInstance(hardwareMap, telemetry);
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
            int teamdiff = 0;
            if(team.equals("red")){
                teamdiff=1;
            }
            if(team.equals("blue")){
                teamdiff=-1;
            }
            /*
            elevator.DebugSpeed=0.6;
            elevator.goUp();
            odometry.goToYolo(10,10,0.5);
            elevator.stop();
            arm.setPosition(0.3);
            arm.updateArm();
            */
            arm.close();
            //point to prop
            odometry.goToYolo(-14*teamdiff,0,0.5,false);
            odometry.rotatePeroMejor(90*teamdiff);
            //place pixel
            odometry.rotatePeroMejor(180);

            odometry.goToYolo(-57*teamdiff,0,0.5,false);

            odometry.goToYolo(0,10,0.5,false);
            //intake.setPower(0.1);
            //utils.waitMs(500, telemetry);
            //intake.setPower(0.2);
            //utils.waitMs(1000,telemetry);
            if(Objects.equals(prop, "1")){
                intake.setPower(0.2);
                utils.waitMs(1500,telemetry);
                //utils.waitMs(2000,telemetry);
                //utils.intakeTime(2000,telemetry,intake,0.2);
                intake.setPower(0);
            }
            if(Objects.equals(prop, "2")){
                odometry.rotatePeroMejor(-90*teamdiff);
                intake.setPower(0.2);
                utils.waitMs(1500,telemetry);
                //utils.waitMs(2000,telemetry);
                //utils.intakeTime(2000,telemetry,intake,0.2);
                intake.setPower(0);
                odometry.rotatePeroMejor(180);
            }
            //intake.setPower(0.5);
            odometry.goToYolo(0,60,0.5,false);
            if(Objects.equals(prop, "3")) {
                intake.setPower(0.3);
                utils.waitMs(1500,telemetry);
                intake.setPower(0);
            }
            if(Objects.equals(robotPosition, "back")) {

                odometry.goToYolo(0, 10, 0.5, false);
                odometry.rotatePeroMejor(0);
                intake.setPower(0.5);
                arm.open();
                odometry.goToYolo(0, 110, 0.5, false);
                //odometry.rotatePeroMejor(0);


                //odometry.goToYolo(0,60,0.5,false);
                arm.close();
                intake.setPower(0);
                odometry.rotatePeroMejor(180);

                //odometry.goToYolo(0, 50, 0.5, false);
                arm.open();
                arm.open();
                arm.open();
                arm.open();
                arm.open();
                odometry.goToYolo(0, -20, 0.3, false);
                arm.close();
                arm.close();
                arm.close();
                arm.close();
                arm.close();
                utils.waitMs(300, telemetry);
                odometry.goToYolo(0,25,0.3,false);
            }else if(robotPosition.equals("front")){

            }


            elevator.DebugSpeed=0.75;

            ElapsedTime notTime=new ElapsedTime();
            notTime.reset();
            notTime.startTime();

            while (-elevator.elevator1.getCurrentPosition()>-1500){
                elevator.goUp();
                telemetry.addData("status","elevating "+"-1300");
                elevator.getPosition(telemetry);
                if(notTime.milliseconds()>4000){
                    try {
                        throw new Exception("amogus");
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }

            elevator.stop();
            //odometry.rotatePeroMejor(180);
            arm.setPosition(0.4);
            arm.updateArm();
            telemetry.update();
            arm.updateArm();
            arm.updateArm();
            arm.updateArm();
            telemetry.update();
            arm.updateArm();
            telemetry.addData("angle",arm.angle);
            telemetry.update();
            utils.waitMs(1000,telemetry);


            if(prop.equals("1")){
                odometry.goToYolo(-20,20,0.5,false);
            }
            if(prop.equals("2")){
                odometry.goToYolo(0,20,0.5,false);
            }
            if(prop.equals("3")){
                odometry.goToYolo(24,20,0.5,false);
            }
            //odometry.goToYolo(0,20,0.5,false);
            utils.waitMs(300,telemetry);
            arm.open();
            telemetry.update();
            arm.open();
            arm.open();
            utils.waitMs(300,telemetry);

            arm.open();
            telemetry.addData("state", "done");
            telemetry.update();
            utils.waitMs(2000,telemetry);


            elevator.DebugSpeed=1;
            while (-elevator.elevator1.getCurrentPosition()>-2000&&false){
                elevator.goUp();
                telemetry.addData("status","elevating "+"-2000");
                elevator.getPosition(telemetry);
                if(notTime.milliseconds()>4000){
                    try {
                        throw new Exception("amogus2");
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            }
            //arm.setZero();
            arm.axis.setPosition(arm.angle-0.2);
            //arm.updateArm();
            utils.waitMs(20000,telemetry);

            //odometry.goToYolo(0,-10,0.5,false);



        }else{
            //arm.close();
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
