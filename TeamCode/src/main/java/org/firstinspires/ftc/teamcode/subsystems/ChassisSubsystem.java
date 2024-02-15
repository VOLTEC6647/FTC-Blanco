package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.utils.calculateRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.info;
import org.firstinspires.ftc.teamcode.subsystems.Parameters;
import org.opencv.core.Mat;

import java.lang.Math;



public class ChassisSubsystem {

    private static ChassisSubsystem instance;
    private HardwareMap hardwareMap;
    public DcMotor FrontLeftMotor;
    public DcMotor FrontRightMotor;
    public DcMotor BackLeftMotor;
    public DcMotor BackRightMotor;

    public boolean REnabled = true;

    double frontRightPower=0;
    double backLeftPower=0;
    double frontLeftPower=0;
    double backRightPower=0;

    public double targetAngle=0;
    private double lastDir;

    private double radians=45;
    
    private Telemetry telemetry;

    private double timeOffset=0;

    int angularError = 0;

    public boolean autonomous = false;

    public boolean snap = false;


    public void rotateAngle(int angle) {

    }

    public Boolean doOverride = false;

    public ChassisSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry = telemetry;
        /*
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        */
        
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FL");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BR");

        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void updateTargetAngle(){
        targetAngle = lastDir;
    }
    public static ChassisSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
          instance = new ChassisSubsystem(hardwareMap,telemetry);
        }
        return instance;
      }
      public void moveY(double speed) {

          frontLeftPower= -speed*(1);
          backLeftPower = -speed*(1);
          backRightPower = -speed*(1);
          frontRightPower =  -speed*(1);

          setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      }

      public void moveX(double speed) {
          frontLeftPower= speed*(1);
          backLeftPower = -speed*(1);
          backRightPower = -speed*(1);
          frontRightPower =  speed*(1);

          setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      }

      public void moveR(double speed) {
          frontLeftPower= -speed*(1);
          backLeftPower = -speed*(1);
          backRightPower = speed*(1);
          frontRightPower = speed*(1);

          setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      }

    private double pidIt(double currentAngle, double r) {
        double Kp = 0.5; // Proportional gain. Adjust this value based on tuning.
        double angularThreshold = autonomous ? 5 : 10; // Different thresholds based on mode
        double angularError = calculateRotation((int) currentAngle, (int) targetAngle);


        // Only adjust if error is above threshold
        if (Math.abs(angularError) > angularThreshold) {
            r = angularError * Kp; // Proportional control

            // For autonomous mode, limit the rotation speed
            if (autonomous) {
                if (r > 0) r = 0.75;
                else if (r < 0) r = -0.75;
            }
        } else {
            r = 0; // No adjustment needed
        }
        return r;
    }

      public void arcadeDrive(double x, double y, double r, double speed, double degrees) {

        telemetry.addData("targetAngle",targetAngle);
        if(Math.abs(targetAngle-180)<45&&snap){
            targetAngle=180;
        }
        if(Math.abs(targetAngle)<45&&snap){
            targetAngle=0;
        }

        if(Math.abs(x)<0.15){
            x=0;
        }
          if(Math.abs(y)<0.15){
              y=0;
          }
          if (REnabled) {
              lastDir = degrees;
              // Condition to decide if immediate correction is needed or if PID should be applied
              if (Math.abs(r) > 0.1 || (System.currentTimeMillis() - timeOffset < 500 && true)) {
                  telemetry.addData("target", targetAngle);
                  telemetry.addData("gyro", degrees);

                  targetAngle = degrees;
                  if (Math.abs(r) > 0.1) {
                      timeOffset = System.currentTimeMillis();
                  }
              } else {
                  // Use the new PID function for proportional control
                  r=pidIt(degrees,r);

                  telemetry.addData("angularError", angularError);
                  telemetry.addData("correction", r); // Show the correction value being applied


              }
          }

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
        //double cosine=Math.round(Math.cos(Math.toRadians(degrees))*1000)/1000;
        //double sine=Math.round(Math.sin(Math.toRadians(degrees))*1000)/1000;
          double cosine=Math.cos(Math.toRadians(degrees));
          double sine=Math.sin(Math.toRadians(degrees));

        xmod = cosine * x - sine * y; // cos(°) sin(°)
        ymod = cosine * y + sine * x;
        telemetry.addData("radians", Math.toRadians(degrees));
        telemetry.addData("cosine", cosine);
        telemetry.addData("sine", sine);
        if(!doOverride){
            x = xmod;
            y = ymod;
        }

        telemetry.addData("xmod", xmod);
        telemetry.addData("ymod", ymod);
        telemetry.addData("pi", Math.PI);
        //x = Math.cos(0.7) * x - Math.sin(0.7) * y; // cos(°) sin(°)
        //y = Math.cos(0.7) * x + Math.sin(0.7) * y;
        frontRightPower=0;
        backLeftPower=0;
        frontLeftPower=0;
        backRightPower=0;


          backLeftPower = speed*(x - y - r);
          backRightPower = speed*(y + x - r);
          frontRightPower =  speed*(y - x - r);
          frontLeftPower= -(y + x + r)*speed;


        if(info.name=="rev"){
            frontLeftPower= -speed*(y - x + r);
            backLeftPower = -speed*(y + x + r);

            backRightPower = -speed*(y + x - r);
            frontRightPower =  -speed*(y - x - r);
        }else{

        }

        if(info.name=="rev"){
            telemetry.addData("NotRobot","Rev");
            //backLeftPower = -speed*(y - x - r);
        }else{

        }

        setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        
}
    public void setMotors(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);

        FrontLeftMotor.setPower(frontLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackLeftMotor.setPower(backLeftPower);
        BackRightMotor.setPower(backRightPower);
    }

    public void setTargetAngle(double deg){
        targetAngle=deg;
        if (targetAngle<0){
            targetAngle=360+targetAngle;
        }
        if (targetAngle>360){
            targetAngle=targetAngle-360;
        }
    }

    public void rotateDeg(double deg){
        targetAngle+=deg;
        if (targetAngle<0){
            targetAngle=360+targetAngle;
        }
        if (targetAngle>360){
            targetAngle=targetAngle-360;
        }
    }

    public static void unaliveInstance() {
        instance = null;
    }

}