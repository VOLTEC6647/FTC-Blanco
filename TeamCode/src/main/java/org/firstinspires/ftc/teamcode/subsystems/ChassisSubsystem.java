package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.utils.calculateRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Chassis;
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


    public void rotateAngle(int angle) {

    }

    public ChassisSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        /*
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        */
        this.telemetry=telemetry;
        
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FL");//
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BR");

        //FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

      public void arcadeDrive(double x, double y, double r, double speed, double degrees) {

        telemetry.addData("targetAngle",targetAngle);
        if(Math.abs(targetAngle-180)<20){
            targetAngle=180;
        }
        if(Math.abs(targetAngle)<20){
            targetAngle=0;
        }

        if(Math.abs(x)<0.15){
            x=0;
        }
          if(Math.abs(y)<0.15){
              y=0;
          }
          if(true) {
              lastDir = degrees;
              if (Math.abs(r)>0.1||(System.currentTimeMillis() - timeOffset < 500&&true)) {

                  telemetry.addData("target",targetAngle);
                  telemetry.addData("gyrro",degrees);

                  targetAngle = degrees;
                    if(Math.abs(r)>0.1){
                        timeOffset = System.currentTimeMillis();
                    }

              }else{
                  angularError = calculateRotation((int) degrees, (int) targetAngle);
                  telemetry.addData("angularError",angularError);
                  double angularThreshold = 10;
                  if(Math.abs(angularError)>angularThreshold){
                      r=angularError*0.5;
                  }
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
        x = xmod;
        y = ymod;
        telemetry.addData("xmod", xmod);
        telemetry.addData("ymod", ymod);
        telemetry.addData("pi", Math.PI);
        //x = Math.cos(0.7) * x - Math.sin(0.7) * y; // cos(°) sin(°)
        //y = Math.cos(0.7) * x + Math.sin(0.7) * y;
        frontRightPower=0;
        backLeftPower=0;
        frontLeftPower=0;
        backRightPower=0;
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



}