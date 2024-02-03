package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous(group = "config")
public class CameraOn extends LinearOpMode {
    @Override
    public void runOpMode() {
        File myFileName = AppUtil.getInstance().getSettingsFile("camera.txt");
        ReadWriteFile.writeFile(myFileName, "on");
    }
}