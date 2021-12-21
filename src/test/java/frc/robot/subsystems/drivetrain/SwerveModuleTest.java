package frc.robot.subsystems.drivetrain;

import org.junit.Assert;
import org.junit.Test;

import static org.junit.Assert.*;

public class SwerveModuleTest {

    @Test
    public void setAngle() {
        double delta = 1e-3; // 1 / 10^3
        SwerveModule module = new SwerveModule(0);
        Assert.assertEquals(-30, module.getclosest(0, 30), delta);
    }
}