package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem {
    private static PivotSubsystem instance;
    public Servo servoPivot;
    public Servo servoClaw;
    private Telemetry telemetry;

    private final double CLAW_OPEN = 1;
    private final double CLAW_CLOSE = 0;
    private final double PIVOT_UP = 0;

    private final double PIVOT_DOWN = 0.5;

    public PivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servoClaw = hardwareMap.get(Servo.class, "claw");
        this.servoPivot = hardwareMap.get(Servo.class, "pivot");
        this.telemetry = telemetry;
    }

    public static PivotSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new PivotSubsystem(hardwareMap, telemetry);
        }

        return instance;
    }

    public void open() {
        servoClaw.setPosition(CLAW_OPEN);
    }

    public void close() {
        servoClaw.setPosition(CLAW_CLOSE);
    }

    public void up() {
        servoPivot.setPosition(PIVOT_UP);
    }

    public void down() {
        servoPivot.setPosition(PIVOT_DOWN);
    }
    public void middle() {
        servoPivot.setPosition(0.54);
    }

    public void pivotControls(boolean buttonA, boolean buttonB, boolean buttonY, boolean leftBumper, boolean rightBumper) {
        if (buttonA) {
            up();
        } else if (buttonB) {
            down();
        } else if (leftBumper) {
            open();
        } else if (rightBumper) {
            close();
        } else if (buttonY) {
            middle();
        }
    }
    public void printPositions() {
        telemetry.addData("claw", servoClaw.getPosition());
        telemetry.addData("pivot", servoPivot.getPosition());
        telemetry.update();
    }



}