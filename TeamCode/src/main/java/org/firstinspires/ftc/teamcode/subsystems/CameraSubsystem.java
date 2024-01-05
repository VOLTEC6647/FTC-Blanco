package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class CameraSubsystem {
    private static HardwareMap hardwareMap;
    private static CameraSubsystem instance;

    private static VisionPortal visionPortal;
    private CameraSubsystem(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;


    }
    public static CameraSubsystem getInstance(HardwareMap hardwareMap){
        if (instance == null) {
            instance = new CameraSubsystem(hardwareMap);
        }
        return instance;
    }
    private void initVisionPortal(){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);



        visionPortal = builder.build();
    }
}
