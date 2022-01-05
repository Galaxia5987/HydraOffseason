package frc.robot.subsystems.drivetrain;

import org.junit.Assert;
import org.junit.Test;

import static org.junit.Assert.*;

public class SwerveModuleTest {

    @Test
    public void getTargetError() {
        double delta = 1e-3; // 1 / 10 ^ 3
        SwerveModule module = new SwerveModule(0);
        Assert.assertEquals(30, module.getTargetError(30, 0), delta);
    }


    public static void main(String[] args) {
        System.out.println();
    }
}