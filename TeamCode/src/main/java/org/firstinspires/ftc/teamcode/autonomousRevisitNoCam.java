package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.subsystems.ODSub;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;

import java.io.File;
import java.util.Objects;

@Autonomous
public class autonomousRevisitNoCam extends LinearOpMode {
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

    private String prop = "";
    private String team = "";
    private String auto = "";
    private String robotPosition = "";

    private String delay = "";

    private String risky = "";

    private String cameras = "";


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

        //camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        //camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        chassis = ChassisSubsystem.getInstance(hardwareMap, telemetry);
        gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        elevator = ElevatorSubsystem.getInstance(hardwareMap, telemetry);
        odometry = OdometrySubsystem.getInstance(hardwareMap, chassis, gyroscope, telemetry);
        arm = ArmSubsystem.getInstance(hardwareMap, telemetry);

        chassis.autonomous = true;

        odometry.resetEncoders();

        gyroscope.reset();

        boolean rotation = true;
        boolean test = true;

        chassis.REnabled = true;

        loadparameters();
        telemetry.update();

        //intake.setPower(0.3);
        //utils.waitMs(2000, telemetry);
        //intake.setPower(0);
        waitForStart();

        //arm.open();


        if (team != "" && robotPosition != "" && prop != "" && auto != "disabled") {
            int teamdiff = 0;
            if (team.equals("red")) {
                teamdiff = 1;
            }
            if (team.equals("blue")) {
                teamdiff = -1;
            }
            if (!delay.equals("")) {
                sleep(Integer.parseInt(delay)* 1000L);
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
            odometry.goToYolo(-17 * teamdiff, 0, 0.5, false);
            odometry.rotatePeroMejor(83 * teamdiff);

            if (cameras.equals("on")) {
                ODSub sub = new ODSub();
                sub.telemetry=telemetry;
                sub.hardwareMap = hardwareMap;
                sub.initTfod();
                sleep(10000);
                sub.telemetryTfod();
                telemetry.update();
                sleep(5000);
                //telemetry.addData("area",camera.area());
                if(sub.lastrecX!=0){
                    if(sub.lastrecX>700){
                        prop="1";
                    }else {
                        prop="2";
                    }
                }else {
                    prop="3";
                }
                telemetry.addData("prop", prop);
                telemetry.update();


            }


            //place pixel
            odometry.rotatePeroMejor(180);

            odometry.goToYolo(-57 * teamdiff, 0, 0.5, false);

            //odometry.goToYolo(0, 10, 0.5, false);
            //intake.setPower(0.1);
            //utils.waitMs(500, telemetry);
            //intake.setPower(0.2);
            //utils.waitMs(1000,telemetry);
            if (Objects.equals(prop, "1")) {
                odometry.goToYolo(0, 7, 0.5, false);
                intake.setPower(0.2);
                utils.waitMs(1500, telemetry);
                //utils.waitMs(2000,telemetry);
                //utils.intakeTime(2000,telemetry,intake,0.2);
                intake.setPower(0);
                odometry.goToYolo(0, 55, 0.5, false);
            }
            if (Objects.equals(prop, "2")) {
                odometry.rotatePeroMejor(-90 * teamdiff);
                intake.setPower(0.2);
                utils.waitMs(1500, telemetry);
                //utils.waitMs(2000,telemetry);
                //utils.intakeTime(2000,telemetry,intake,0.2);
                intake.setPower(0);
                odometry.rotatePeroMejor(180);
                odometry.goToYolo(0, 60, 0.5, false);

            }
            //intake.setPower(0.5);
            odometry.goToYolo(0, 60, 0.5, false);
            if (Objects.equals(prop, "3")) {
                intake.setPower(0.3);
                utils.waitMs(1500, telemetry);
                intake.setPower(0);
            }
            if (risky.equals("yes")) {
                if (Objects.equals(robotPosition, "back")) {


                    odometry.goToYolo(0, 40, 0.3, false);

                    sleep(5000);
                } else if (robotPosition.equals("front")) {
                    odometry.goToYolo(0, 20, 0.5, false);
                }


                elevator.DebugSpeed = 0.75;

                ElapsedTime notTime = new ElapsedTime();
                notTime.reset();
                notTime.startTime();

                while (-elevator.elevator1.getCurrentPosition() > -1500) {
                    elevator.goUp();
                    telemetry.addData("status", "elevating " + "-1300");
                    elevator.getPosition(telemetry);
                    if (notTime.milliseconds() > 4000) {
                        try {
                            throw new Exception("amogus");
                        } catch (Exception e) {
                            throw new RuntimeException(e);
                        }
                    }
                }

                elevator.stop();
                sleep(5000);
                //odometry.rotatePeroMejor(180);

                arm.setPosition(1);

                sleep(2000);


                if (prop.equals("3")) {
                    odometry.goToYolo(-15, 26, 0.5, false);
                }
                if (prop.equals("2")) {
                    odometry.goToYolo(0, 26, 0.5, false);
                }
                if (prop.equals("1")) {
                    odometry.goToYolo(15, 26, 0.5, false);
                }
                //odometry.goToYolo(0,20,0.5,false);

                sleep(500);

                arm.open();
                telemetry.addData("state", "done");
                telemetry.update();
                utils.waitMs(2000, telemetry);


                elevator.DebugSpeed = 1;
                while (-elevator.elevator1.getCurrentPosition() > -2000 && false) {
                    elevator.goUp();
                    telemetry.addData("status", "elevating " + "-2000");
                    elevator.getPosition(telemetry);
                    if (notTime.milliseconds() > 4000) {
                        try {
                            throw new Exception("amogus2");
                        } catch (Exception e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
                //arm.setZero();

                //arm.axis.setPosition(arm.axis.MAX_POSITION - 0.2);
                //arm.updateArm();

                odometry.goToYolo(0,-10,0.5,false);

            } else {
                if (robotPosition.equals("front")) {
                    odometry.goToYolo(0, 15, 0.5, false);
                }
                if (robotPosition.equals("back")) {
                    odometry.goToYolo(0, 90, 0.5, false);
                }
            }

        } else {
            //arm.close();
            //odometry.goToYolo(0,-64,0.5);

            //odometry.goToYolo(-125,0,0.5);
            //odometry.goToYolo(-222,0,0.5);

            //chassis.targetAngle+=;
            //odometry.goToYolo(0,-20,0.3);
        }
        chassis.setMotors(0, 0, 0, 0);
        while (opModeIsActive()) {
            String amogus = "yes";
        }

    }

    private void loadparameters() {
        File myFileName;

        myFileName = AppUtil.getInstance().getSettingsFile("prop.txt");
        prop = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        team = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("auto.txt");
        auto = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("position.txt");
        robotPosition = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("delay.txt");
        delay = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("risky.txt");
        risky = ReadWriteFile.readFile(myFileName);

        myFileName = AppUtil.getInstance().getSettingsFile("camera.txt");
        cameras = ReadWriteFile.readFile(myFileName);

        telemetry.addData("Prop", prop);
        telemetry.addData("Team", team);
        telemetry.addData("Auto", auto);
        telemetry.addData("Position", robotPosition);
        telemetry.addData("Delay", delay);
        telemetry.addData("Risky", risky);
        telemetry.addData("Camera", cameras);


    }
}
