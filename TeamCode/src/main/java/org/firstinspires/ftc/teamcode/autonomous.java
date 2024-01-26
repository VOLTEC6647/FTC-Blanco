package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@Autonomous
public class autonomous extends LinearOpMode {
    private OpenCVSubsystem camera;
    private ChassisSubsystem chassis;
    private GyroscopeSubsystem gyroscope;
    private ElevatorSubsystem elevator;
    private OdometrySubsystem odometry;
    private ArmSubsystem arm;
    private PivotSubsystem pivot;
    private Blinker control_Hub;
    private double speed = 1;
    private IMU imu;
    private double baseSpeed = 0.5;
    private Orientation orientation;
    private DcMotor intake;
    boolean hasTarget = false;
    public double gyr;

    @Override
    public void runOpMode() {

        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(IMU.class, "imu");
        if (Parameters.robot == "marvin") {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }

        camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        chassis = ChassisSubsystem.getInstance(hardwareMap, telemetry);
        gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        elevator = ElevatorSubsystem.getInstance(hardwareMap);
        odometry = OdometrySubsystem.getInstance(hardwareMap, chassis, telemetry);
        arm = ArmSubsystem.getInstance(hardwareMap, telemetry);
        pivot = PivotSubsystem.getInstance(hardwareMap, telemetry);

        odometry.resetEncoders();
        //gyroscope.reset();

        waitForStart();
        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////
            //////////////////////REV AUTONOMOUS///////////////////////
            ///////////////////////////////////////////////////////////
            boolean dolor = true;

            int objectSide = camera.findObjectSide();

            /*if (info.name == "popo"); {
//                pivot.close();
//                sleep(50);
//                pivot.up();
//                elevator.goUp();
                odometry.resetEncoders();

                switch (objectSide) {
                    case 0:
                        telemetry.addData("OBJECT NOT FOUND. ", "SKILL ISSUE IM AFRAID");
                        sleep(5000);
                        break;

                    case 1:
                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(0,-63);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while(dolor) {
                            dolor = odometry.rotateTo(gyroscope.getRotation(), -90);
                        }
                        chassis.setMotors(0,0,0,0);
                        gyroscope.reset();
                        odometry.resetEncoders();

                        sleep(300);
                        pivot.up();
                        pivot.open();

                        dolor = true;
                        while(dolor) {
                            dolor = odometry.goTo(20,0);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while(dolor) {
                            dolor = odometry.goTo(0,-100);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();
                        
                        break;
                    case 2:
                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(0,-63);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(10,0);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        sleep(300);
                        pivot.open();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(-20,0);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(0,-20);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(-100,0);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        break;
                    case 3:
                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(0,-63);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while(dolor) {
                            dolor = odometry.rotateTo(gyroscope.getRotation(), 90);
                        }
                        chassis.setMotors(0,0,0,0);
                        gyroscope.reset();
                        odometry.resetEncoders();

                        sleep(300);
                        pivot.up();
                        pivot.open();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(20,0);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        dolor = true;
                        while (dolor) {
                            dolor = odometry.goTo(0,-50);
                        }
                        chassis.setMotors(0,0,0,0);
                        odometry.resetEncoders();

                        break;
                }*/


                ///////////////////////////////////////////////////////////
                /////////            GOBILDA AUTONOMOUS          //////////
                ///////////////////////////////////////////////////////////

                /////////////         BLUE TEAM               /////////////

                if (info.name == "gobilda") {
                    arm.close();
                    arm.setPosition(1);
                    arm.updateArm(0);
                    elevator.goUp();

                    odometry.resetEncoders();

                    switch (objectSide) {
                        case 0:
                            telemetry.addData("OBJECT NOT FOUND. ", "SKILL ISSUE IM AFRAID");
                            sleep(5000);
                            break;

                        case 1:
                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(0,-63);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while(dolor) {
                                dolor = odometry.rotateTo(gyroscope.getRotation(), -90);
                            }
                            chassis.setMotors(0,0,0,0);
                            gyroscope.reset();
                            odometry.resetEncoders();

                            sleep(300);
                            arm.setPosition(1);
                            arm.updateArm(1);

                            dolor = true;
                            while(dolor) {
                                dolor = odometry.goTo(20,0);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while(dolor) {
                                dolor = odometry.goTo(0,-100);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            break;
                        case 2:
                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(0,-63);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(10,0);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            sleep(300);
                            pivot.open();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(-20,0);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(0,-20);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(-100,0);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            break;
                        case 3:
                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(0,-63);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while(dolor) {
                                dolor = odometry.rotateTo(gyroscope.getRotation(), 90);
                            }
                            chassis.setMotors(0,0,0,0);
                            gyroscope.reset();
                            odometry.resetEncoders();

                            sleep(300);
                            pivot.up();
                            pivot.open();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(20,0);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            dolor = true;
                            while (dolor) {
                                dolor = odometry.goTo(0,-50);
                            }
                            chassis.setMotors(0,0,0,0);
                            odometry.resetEncoders();

                            break;
                    }

                }
            elevator.goUp();
               telemetry.addData("object side: ", objectSide);

               telemetry.update();
            }


        }
    }
//}


