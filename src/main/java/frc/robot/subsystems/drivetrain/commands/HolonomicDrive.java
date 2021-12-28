package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrives;
    private final XboxController xboxController = RobotContainer.xboxController;
    private double rightY;
    private double leftY;


    public HolonomicDrive(SwerveModule swerveModules, SwerveDrive swerveDrives) {
        this.swerveDrives = swerveDrives;
        addRequirements(swerveDrives);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveDrives.drive(Constants.Swerve.FORWARD, Constants.Swerve.STRAFE, Constants.Swerve.ROTATION);
        rightY = xboxController.getY(GenericHID.Hand.kRight);
        leftY = xboxController.getY(GenericHID.Hand.kLeft);
    }

    @Override
    public boolean isFinished() {
        return isFinished();
    }

    @Override
    public void end(boolean interrupted) {

    }
}