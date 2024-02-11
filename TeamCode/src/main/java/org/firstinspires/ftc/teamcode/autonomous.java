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

@Autonomous
public class autonomous extends LinearOpMode {
    private OpenCVSubsystem camera;
    private ChassisSubsystem chassis;
    private GyroscopeSubsystem gyroscope;
    private ElevatorSubsystem elevator;
    private OdometrySubsystem odometry;
    private ArmSubsystem arm;
    private Blinker control_Hub;
    private double speed = 1;
    private IMU imu;
    private double baseSpeed = 0.5;
    private Orientation orientation;
    private DcMotor intake;
    boolean hasTarget = false;
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
        //imu = hardwareMap.get(IMU.class, "imu");
        if (Parameters.robot == "marvin") {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }

        camera = OpenCVSubsystem.getInstance(hardwareMap, telemetry);
        chassis = ChassisSubsystem.getInstance(hardwareMap,telemetry);
        gyroscope = GyroscopeSubsystem.getInstance(hardwareMap);
        elevator = ElevatorSubsystem.getInstance(hardwareMap, telemetry);
        odometry = OdometrySubsystem.getInstance(hardwareMap, chassis, telemetry);
        arm = ArmSubsystem.getInstance(hardwareMap, telemetry);


        odometry.resetEncoders();

        gyroscope.reset();

        boolean rotation = true;
        boolean test = true;

        waitForStart();
        while (opModeIsActive()) {
            boolean dolor = true;


            odometry.resetEncoders();


            chassis.setMotors(0,0,0,0);
            sleep(500);

            dolor = true;
            while (dolor) {
                dolor = odometry.rotateTo(gyroscope.getRotation(), -90);
            }
            odometry.resetEncoders();
            break;

//            if (gyroscope.getRotation() < -88) {
//                arm.open();
//                sleep(1000);
//                arm.setPosition(1);
//            }
//                chassis.setMotors(0,0,0,0);
//
//            //arm.close();
            //arm.setPosition(0);
            //elevator.goUp();

            /*switch (camera.findObjectSide()) {
                case 1:
                    odometry.goTo(0,63);
                    odometry.rotateTo(gyroscope.getRotation(), -90);
                    if (gyroscope.getRotation() < -88) {
                        arm.open();
                        sleep(1000);
                        arm.setPosition(1);
                    }

                break;

                case 2:
                    odometry.goTo(-15,63);
                    if (odometry.getXDist() > 22) {
                        arm.open();
                        arm.setPosition(1);
                    }

                break;

                case 3:
                    odometry.goTo(0,63);
                    chassis.targetAngle = 90;
                    if (gyroscope.getRotation() < 88) {
                        arm.open();
                        sleep(1000);
                        arm.setPosition(1);
                    }

                break;
            }*/

            //telemetry.addData("cx: ", camera.getcx());
            //telemetry.addData("cy: ", camera.getcy());
//            odometry.printEncoders();
//            telemetry.addData("rotation: ", gyroscope.getRotation());
//            telemetry.update();

        }

    }
}
