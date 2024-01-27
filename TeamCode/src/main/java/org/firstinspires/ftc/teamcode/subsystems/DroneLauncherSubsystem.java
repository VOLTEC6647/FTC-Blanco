package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncherSubsystem {
    private Servo launcher;
    private Telemetry telemetry;
    private static DroneLauncherSubsystem instance;
    public static DroneLauncherSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new DroneLauncherSubsystem(hardwareMap,telemetry);
        }
        return instance;

    }
    public DroneLauncherSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        launcher = hardwareMap.get(Servo.class, "launcher");
        this.telemetry=telemetry;

    }

    //POSSIBLY SWAP MAX_POSITION AND MIN_POSITION OUT
    public void launch(){
        launcher.setPosition(launcher.MAX_POSITION);
        telemetry.addData("launcher",launcher.getPosition());
    }
    public void reset(){launcher.setPosition(launcher.MIN_POSITION);
    telemetry.addData("launcher",launcher.getPosition());

    }
}
