package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.kauailabs.navx.ftc.AHRS;

public class GyroscopeSubsystem {

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private AHRS navx;
    private static GyroscopeSubsystem instance;
    private int offset;

    
    public static GyroscopeSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new GyroscopeSubsystem(hardwareMap);
        }
        return instance;
    }
    public GyroscopeSubsystem(HardwareMap hardwareMap){
        navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        navx.zeroYaw();
    }
    public double getRotation(){
        return navx.getYaw()+offset;
    }

    public void reset(){
        navx.zeroYaw();
    }
    public void setOffset(int degrees){
        offset = degrees;
    }


}