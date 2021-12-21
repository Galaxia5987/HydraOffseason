package frc.robot.subsystems.drivetrain;

import org.junit.Assert;
import org.junit.Test;

import static org.junit.Assert.*;

public class SwerveModuleTest {

    @Test
    public void setAngle() {
        double delta = 1e-3; // 1 / 10^3
        SwerveModule module = new SwerveModule(0);
        Assert.assertEquals(90, module.getclosest(270, 0), delta);
        Assert.assertEquals(30, module.getclosest(30, 0), delta);
        Assert.assertEquals(120, module.getclosest(30, 270), delta);
    }
}