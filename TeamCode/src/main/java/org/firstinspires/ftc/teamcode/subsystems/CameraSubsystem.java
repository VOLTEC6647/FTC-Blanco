package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class CameraSubsystem {
    private static HardwareMap hardwareMap;
    private static CameraSubsystem instance;

    private static VisionPortal visionPortal;

    private TfodProcessor tfod;
    private CameraSubsystem(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        initVisionPortal();

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

        tfod = TfodProcessor.easyCreateWithDefaults();

        builder.addProcessor(tfod);

        visionPortal = builder.build();

        getFrame();
        averageColor("blue", 10);

    }
    private void getFrame() {
        visionPortal.saveNextFrameRaw("frame.png"); // Assuming this returns the file path

    }

    private void averageColor(String color, int threshold) {
        String filePath = "frame.png";
        Bitmap bitmap = BitmapFactory.decodeFile(filePath);

        if (bitmap != null) {
            long totalColorValue = 0;
            int count = 0;
            int width = bitmap.getWidth();
            int height = bitmap.getHeight();

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    int pixel = bitmap.getPixel(x, y);
                    int value = 0;

                    switch (color.toLowerCase()) {
                        case "red":
                            value = (pixel >> 16) & 0xff;
                            break;
                        case "green":
                            value = (pixel >> 8) & 0xff;
                            break;
                        case "blue":
                            value = pixel & 0xff;
                            break;
                        default:
                            throw new IllegalArgumentException("Invalid color: " + color);
                    }

                    if (value >= threshold) {
                        totalColorValue += value;
                        count++;
                    }
                }
            }

            if (count > 0) {
                float averageColorValue = totalColorValue / (float) count;
                System.out.println("Average " + color + " Value: " + averageColorValue);
            } else {
                System.out.println("No pixels met the threshold for " + color);
            }
        }
    }
}
