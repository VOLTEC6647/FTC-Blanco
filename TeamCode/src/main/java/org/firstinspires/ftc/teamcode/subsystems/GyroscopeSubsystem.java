package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GyroscopeSubsystem {

    private static GyroscopeSubsystem instance;
    
    public static GyroscopeSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new GyroscopeSubsystem(hardwareMap);
        }
        return instance;
    }
    public GyroscopeSubsystem(HardwareMap hardwareMap){
        
    }
    
}