package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncherSubsystem {
    private Servo launcher;
    private static DroneLauncherSubsystem instance;
    public static DroneLauncherSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new DroneLauncherSubsystem(hardwareMap);
        }
        return instance;

    }

    public DroneLauncherSubsystem(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, "launcher");
    }
    public void launch(){
        launcher.setPosition(launcher.getPosition()+10);
    }
}
