package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous(group = "config", name = "leoPark")
public class doPark extends LinearOpMode {
    @Override
    public void runOpMode() {
        File myFileName = AppUtil.getInstance().getSettingsFile("dopark.txt");
        ReadWriteFile.writeFile(myFileName, "yes");
    }
}