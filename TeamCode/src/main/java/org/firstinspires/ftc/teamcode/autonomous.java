package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroscopeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;

@Autonomous
public class autonomous extends LinearOpMode {
    private OpenCVSubsystem openCVSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;
    private ChassisSubsystem chassisSubsystem;
    private GyroscopeSubsystem gyroscopeSubsystem;
    //private ElevatorSubsystem elevatorSubsystem;
    //private OdometrySubsystem odometrySubsystem;
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
        //this.telemetry = telemetry;
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(IMU.class, "imu");
        if (Parameters.robot == "marvin") {
            intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        }

        openCVSubsystem = OpenCVSubsystem.getInstance(hardwareMap);
        aprilTagSubsystem = AprilTagSubsystem.getInstance(hardwareMap,telemetry);
        chassisSubsystem = ChassisSubsystem.getInstance(hardwareMap,telemetry);
        gyroscopeSubsystem = GyroscopeSubsystem.getInstance(hardwareMap);
        //elevatorSubsystem = ElevatorSubsystem.getInstance(hardwareMap);
        //odometrySubsystem = OdometrySubsystem.getInstance(hardwareMap, chassisSubsystem);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            //telemetry.addData("Width: ", openCVSubsystem.width);
            telemetry.update();
        }
    }
}
