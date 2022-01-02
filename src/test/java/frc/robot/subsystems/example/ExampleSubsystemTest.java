package frc.robot.subsystems.example;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.utils.Utils;
import org.junit.Test;

import static org.junit.Assert.*;

public class ExampleSubsystemTest {

    /**
     * Sets the angle for the module.
     *
     * @param angle is the angle to set the module to. [deg]
     */
    public static int calcAngle(double angle, double currAngle) {
        currAngle %= 360;
        currAngle = (currAngle < 0) ? (360 + currAngle) : currAngle;

        angle %= 360;
        angle = (angle < 0) ? (360 + angle) : angle;

        System.out.println("Required angle: " + angle + " Current angle: " + currAngle);

        double error = angle - currAngle;
        error = (Math.abs(error) > 180) ? (error - (360 * Math.signum(error))) : error;
        error %= 360;
        error = Utils.checkDeadband(error, 0.01);

        return (int)error;
    }

    public static void main(String[] args) {
        System.out.println("Expected: " + 25 + ", actual: " + calcAngle(30,5));
        System.out.println("Expected: " + -90 + ", actual: " + calcAngle(-45,45));
        System.out.println("Expected: " + 5 + ", actual: " + calcAngle(-45,-50));
        System.out.println("Expected: " + -5 + ", actual: " + calcAngle(-50,-45));
        System.out.println("Expected: " + 0 + ", actual: " + calcAngle(1,1));
        System.out.println("Expected: " + 1 + ", actual: " + calcAngle(-179,180));
        System.out.println("Expected: " + 0 + ", actual: " + calcAngle(-180,180));
        System.out.println("Expected: " + 1 + ", actual: " + calcAngle(0,-1));
        System.out.println("Expected: " + -1 + ", actual: " + calcAngle(-1,0));
        System.out.println("Expected: " + -1 + ", actual: " + calcAngle(0,1));
        System.out.println("Expected: " + 1 + ", actual: " + calcAngle(1,0));
        System.out.println("Expected: " + 0 + ", actual: " + calcAngle(0,0));
        System.out.println("Expected: " + 0 + ", actual: " + calcAngle(-180,-180));
        System.out.println("Expected: " + 179 + ", actual: " + calcAngle(-1,180));
        System.out.println("Expected: " + 0 + ", actual: " + calcAngle(180,-180));
        System.out.println("Expected: " + 30 + ", actual: " + calcAngle(-60,-90));
        System.out.println("Expected: " + 150 + ", actual: " + calcAngle(90,-60));
        System.out.println("Expected: " + -130 + ", actual: " + calcAngle(170,-60));
        System.out.println("Expected: " + -1 + ", actual: " + calcAngle(180,-179));
        System.out.println("Expected: " + -1 + ", actual: " + calcAngle(180,-179 + 360 * 4));
    }
}