package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

import java.util.function.DoubleSupplier;

/**
 * Get as a parameter- swerveDrive, i, desiredAngle.
 */
public class Rotate extends CommandBase {
    private final SwerveDrive swerveDrive;
    private int i;
    private DoubleSupplier desiredAngle;

    public Rotate(SwerveDrive swerveDrive, int i, DoubleSupplier desiredAngle){
        this.swerveDrive = swerveDrive;
        this.i = i;
        this.desiredAngle = desiredAngle;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        desiredAngle.getAsDouble();
        swerveDrive.getModule(i).setAngle(desiredAngle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    swerveDrive.getModule(i).stop();
    }
}
