package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;

import java.lang.Math;



public class ChassisSubsystem {

    private static ChassisSubsystem instance;
    private HardwareMap hardwareMap;
    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;
    
    private double radians=45;
    
    private Telemetry telemetry;
    
    
    
    
    

    public ChassisSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        /*
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        */
        this.telemetry=telemetry;
        
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
    }
    public static ChassisSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
          instance = new ChassisSubsystem(hardwareMap,telemetry);
        }
        return instance;
      }



      public void arcadeDrive(double x, double y, double r, double speed, double degrees) {

        //telemetry.addData("x", x);
        //telemetry.addData("y", y);
        //degrees=degrees+180;

        if(degrees<0){
            degrees=(180+(180-Math.abs(degrees)));
        }
        telemetry.addData("degrees", degrees);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        double xmod;
        double ymod;
        double cosine=Math.round(Math.cos(Math.toRadians(180))*1000)/1000;
        double sine=Math.round(Math.sin(Math.toRadians(180))*1000)/1000;

        xmod = cosine * x - sine * y; // cos(°) sin(°)
        ymod = cosine * y + sine * x;
        telemetry.addData("radians", Math.toRadians(90));
        telemetry.addData("cosine", cosine);
        telemetry.addData("sine", sine);
        x = xmod;
        y = ymod;
        telemetry.addData("xmod", xmod);
        telemetry.addData("ymod", ymod);
        telemetry.addData("pi", Math.PI);
        //x = Math.cos(0.7) * x - Math.sin(0.7) * y; // cos(°) sin(°)
        //y = Math.cos(0.7) * x + Math.sin(0.7) * y;
        double frontRightPower=0;
        double backLeftPower=0;
        double frontLeftPower=0;
        double backRightPower=0;
        if(Parameters.robot=="marvin"){
            frontLeftPower=-speed*(y + x - r);
        }else{

        }
        // = /*-*/speed*(y + x - r);




        if(Parameters.robot=="marvin"){
            backLeftPower = -speed*(y - x - r);
        }else{

        }

        backLeftPower = speed*(x - y - r);
        backRightPower = speed*(y + x - r);
        frontRightPower =  speed*(y - x - r);
        frontLeftPower= -(y + x + r)*speed;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);

        FrontLeftMotor.setPower(frontLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackLeftMotor.setPower(backLeftPower);
        BackRightMotor.setPower(backRightPower);

        
}

    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    // Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
    /*MecanumDriveOdometry m_odometry = new MecanumDriveOdometry
            (
                    m_kinematics, mak,
                    new Pose2d(5.0, 13.5, new Rotation2d())
    );*/

}