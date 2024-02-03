package org.firstinspires.ftc.teamcode.subsystems;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVSubsystemmm {


    public HardwareMap hardwareMap;
    public  Telemetry telemetry;
    public OpenCVSubsystemmm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        initOpenCV();
        this.telemetry=telemetry;
    }
    public static OpenCVSubsystemmm getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        OpenCVSubsystemmm instance = new OpenCVSubsystemmm(hardwareMap, telemetry);
        if (instance == null) {
        }
        return instance;
    }
    double cX = 0;
    double cY = 0;
    public double width = 0;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(new PropDetectionPipeline());

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(640,480,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //telemetry.addData("errorFUCK", errorCode);
            }
        });
        //controlHubCam.openCameraDevice();
        //telemetry.addLine("Waiting for start");
        //telemetry.update();
    }
    public static double cx;
    public static double cy;

    public class PropDetectionPipeline extends OpenCvPipeline{
        Mat mat = new Mat();
        Mat sleek = new Mat();
        Moments moment = new Moments();
        Mat bgrImage = new Mat();

        @Override
        public Mat processFrame(Mat input){
            //Mat mat = input.clone();
            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
            Scalar lowerBlue = new Scalar(94,120,140,0);
            Scalar upperBlue = new Scalar(108,255,255,255);
            Core.inRange(mat,lowerBlue,upperBlue,mat);
            //kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1,1));
            //Imgproc.morphologyEx(mat,mat,Imgproc.MORPH_OPEN, kernel);
            //kernel.release();
            //Mat[] hsvChannels = new Mat[3];
            //Core.split(mat, Arrays.asList(hsvChannels));
            moment = Imgproc.moments(mat);
            OpenCVSubsystemmm.cx = moment.get_m10()/moment.get_m00();
            OpenCVSubsystemmm.cy = moment.get_m01()/moment.get_m00();
            Point centroid = new Point(cx,cy);
            Imgproc.cvtColor(mat,bgrImage, Imgproc.COLOR_GRAY2RGB);
            Imgproc.line(bgrImage,centroid,centroid,new Scalar(0,255,0),10);
            //Imgproc.rectangle(input
            //  new Point(input.cols()/4,));
            return bgrImage;
        }

    }

    final int LEFTSIDE = 1;
    final int CENTERSIDE = 2;
    final int RIGHTSIDE = 3;

    public int findObjectSide() {
        int side = 0;
        if (getcx() < 220) {
            side = LEFTSIDE;
            controlHubCam.stopStreaming();
        } else if (getcx() > 220 && getcx() < 440) {
            side = CENTERSIDE;
            controlHubCam.stopStreaming();
        } else if (getcx() > 440) {
            side = RIGHTSIDE;
            controlHubCam.stopStreaming();
        }

        return side;
    }
    public double getcx() {
        return cx;
    }

    public double getcy() {
        return cy;
    }

}