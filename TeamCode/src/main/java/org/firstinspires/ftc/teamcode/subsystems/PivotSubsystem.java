package org.firstinspires.ftc.teamcode.subsystems;

import android.icu.text.IDNA;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem {
    private static PivotSubsystem instance;
    public Servo servoPivot;
    public Servo servoClaw;
    public Servo axis;
    private Telemetry telemetry;

    private final double CLAW_OPEN = 1;
    private final double CLAW_CLOSE = 0;
    private final double PIVOT_UP = 1;
    private final double PIVOT_PICKUP = 0.3;
    private final double PIVOT_DOWN = 0;

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

    public void pickup() {
        servoPivot.setPosition(PIVOT_PICKUP);
    }

    public void pivotControls(boolean buttonA, boolean buttonB, boolean buttonX, boolean leftBumper, boolean rightBumper) {
        if (buttonA) {
            pickup();
        } else if (buttonB) {
            up();
        } else if (rightBumper) {
            open();
        } else if (leftBumper) {
            close();
        } else if (buttonX) {
            down();
        }
    }
    public void printPositions() {
        telemetry.addData("claw", servoClaw.getPosition());
        telemetry.addData("pivot", servoPivot.getPosition());
        telemetry.update();
    }
}