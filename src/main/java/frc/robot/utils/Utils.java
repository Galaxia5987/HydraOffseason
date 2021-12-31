package frc.robot.utils;

public class Utils {

    /**
     * Checks whether a value is in a specified dead band.
     *
     * @param val      is the value.
     * @param deadBand is the dead band that the function checks.
     * @return 0 or the input val (depending on the dead band).
     */
    public static double checkDeadband(double val, double deadBand) {
        return (Math.abs(val) < deadBand) ? 0 : val;
    }
}
