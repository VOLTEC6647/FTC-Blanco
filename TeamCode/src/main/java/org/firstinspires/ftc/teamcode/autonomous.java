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
    private double baseSpeed = 1;
    private Orientation orientation;
    private DcMotor intake;
    boolean hasTarget = false;
    public double gyr;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");

        camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        chassis = ChassisSubsystem.getInstance(hardwareMap, telemetry);
        gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        elevator = ElevatorSubsystem.getInstance(hardwareMap, telemetry);
        odometry = OdometrySubsystem.getInstance(hardwareMap, chassis, telemetry);
        pivot = PivotSubsystem.getInstance(hardwareMap, telemetry);

        odometry.resetEncoders();
        //gyroscope.reset();

        waitForStart();
        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////
            //////////////////////REV AUTONOMOUS///////////////////////
            ///////////////////////////////////////////////////////////
            boolean dolor = true;

                final int objectSide = camera.findObjectSide();

                pivot.close();
                sleep(50);
                pivot.down();
                odometry.resetEncoders();

                if (objectSide == 1) {
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
                    pivot.middle();
                    sleep(1000);
                    pivot.open();
                    sleep(1000);
                    pivot.down();

                    dolor = true;
                    while(dolor) {
                        dolor = odometry.goTo(0,20);
                    }
                    chassis.setMotors(0,0,0,0);
                    odometry.resetEncoders();

                    dolor = true;
                    while(dolor) {
                        dolor = odometry.goTo(-60,0);
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
                } else if (objectSide == 2) {
                    dolor = true;
                    while (dolor) {
                        dolor = odometry.goTo(0,-63);
                    }
                    chassis.setMotors(0,0,0,0);
                    odometry.resetEncoders();

                    dolor = true;
                    while(dolor) {
                        dolor = odometry.goTo(5,0);
                    }
                    chassis.setMotors(0,0,0,0);
                    odometry.resetEncoders();

                    sleep(300);
                    pivot.middle();
                    sleep(1000);
                    pivot.open();
                    sleep(1000);
                    pivot.down();

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
                } else if (objectSide == 3) {
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
                    pivot.middle();
                    sleep(1000);
                    pivot.open();
                    sleep(1000);
                    pivot.down();

                    dolor = true;
                    while(dolor) {
                        dolor = odometry.goTo(-20,0);
                    }
                    chassis.setMotors(0,0,0,0);
                    odometry.resetEncoders();

                    dolor = true;
                    while(dolor) {
                        dolor = odometry.goTo(0,100);
                    }
                    chassis.setMotors(0,0,0,0);
                    odometry.resetEncoders();

                    break;
                }

            //elevator.goUp();
            telemetry.addData("object side: ", camera.findObjectSide());

            telemetry.update();
        }


    }
}

