package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

/**
 *  Get- swerveDrive, xboxController, i.
 */
public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrives;
    private final XboxController xboxController = RobotContainer.xboxController;
    private int module;

    public HolonomicDrive(SwerveDrive swerveDrives, int module) {
        this.swerveDrives = swerveDrives;
        this.module = module;
        addRequirements(swerveDrives);
    }

    @Override
    public void execute() {
        double forward = xboxController.getY(GenericHID.Hand.kLeft) * Constants.Swerve.MAX_VELOCITY;
        double strafe = xboxController.getX(GenericHID.Hand.kRight) * Constants.Swerve.MAX_VELOCITY;
        double rotation = xboxController.getX(GenericHID.Hand.kLeft) * Constants.Swerve.MAX_ROTATION;

        if (Math.abs(forward)<= Constants.Swerve.DEADBAND * Constants.Swerve.MAX_VELOCITY){
            forward = 0;
        }
        if (Math.abs(strafe) <= Constants.Swerve.DEADBAND * Constants.Swerve.MAX_VELOCITY){
            strafe = 0;
        }
        if (Math.abs(rotation)<= Constants.Swerve.DEADBAND * Constants.Swerve.MAX_ROTATION){
            rotation = 0;
        }
        
        swerveDrives.drive(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrives.getModule(module).stop();
    }
}