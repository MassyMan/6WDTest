package org.firstinspires.ftc.teamcode.SWD;

public class Utils {

    /**
     * deltaAngleClip is determines the difference in current robot heading and target heading,
     * normalizing it to a maximum of 180 degrees. (will either be left or right turn of <= 180 deg.)
     * to eliminate inefficient routes in 6WD pathing.
     *
     * Example Usage: deltaAngleClip(targetHeading - currentHeading)
     */
    public double deltaAngleClip(double value) {
        if (value > 180) {
            value -= 360;
        } else if (value < -180) {
            value += 360;
        } else {
            return value;
        }
        return value;
    }
}
