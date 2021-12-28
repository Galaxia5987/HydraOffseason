package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class HolonomicDrive extends CommandBase {
    private final SwerveModule [] swerveModules;
    private final SwerveDrive swerveDrives;

    public HolonomicDrive(SwerveModule[] swerveModules, SwerveDrive swerveDrives) {
        this.swerveModules = swerveModules;
        this.swerveDrives = swerveDrives;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished();
    }
}

