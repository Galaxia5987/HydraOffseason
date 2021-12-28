package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class Rotate extends CommandBase {
    private final SwerveDrive swerveDrive;
    private int [] i = {0, 1, 2, 3};

    public Rotate(SwerveModule swerveModule, SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        this.i = i;
        addRequirements(swerveDrive);
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
        return false;
    }
}
