package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

public class PositionTrackerSubsystem {
    private static PositionTrackerSubsystem instance;
    public PositionTrackerSubsystem(HardwareMap hardwareMap){
        
    }
    public static PositionTrackerSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new PositionTrackerSubsystem(hardwareMap);
        }
        return instance;
    }
}