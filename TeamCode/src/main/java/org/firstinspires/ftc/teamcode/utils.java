package org.firstinspires.ftc.teamcode;

public class utils {
    public static int shortestPathDirection(int angleA, int angleB) {
        int difference = Math.abs(angleA - angleB);
        int shortestPath = Math.min(difference, 360 - difference);

        // Determine direction
        if ((angleB - angleA + 360) % 360 == shortestPath) {
            return 1;
        } else {
            return -1;
        }
    }
    public static int calculateRotation(int angleA, int angleB) {
        int forwardRotation = (angleB - angleA + 360) % 360;
        int backwardRotation = (angleA - angleB + 360) % 360;

        // Determine if forward rotation is shorter, otherwise use backward
        if (forwardRotation <= backwardRotation) {
            return forwardRotation;
        } else {
            return -backwardRotation;
        }
    }
}
