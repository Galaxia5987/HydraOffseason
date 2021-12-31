package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import static frc.robot.Constants.SwerveDrive.*;
import static frc.robot.Ports.Controls.JOYSTICK_LEFT;
import static frc.robot.Ports.Controls.JOYSTICK_RIGHT;

public class SwerveDefaultCommand extends CommandBase {
    private final Joystick joystickRight = new Joystick(JOYSTICK_RIGHT);
    private final Joystick joystickLeft = new Joystick(JOYSTICK_LEFT);
    private final SwerveDrive swerveDrive;

    /**
     * Constructor
     *
     * @param swerveDrive is the swerve drive
     */
    public SwerveDefaultCommand(SwerveDrive swerveDrive) {
        addRequirements(swerveDrive);
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        swerveDrive.setFieldOriented(true);
    }

    /**
     * Sets the states in a loop according to the joystick double suppliers.
     */
    @Override
    public void execute() {
        swerveDrive.holonomicDrive(MAX_VELOCITY * Utils.checkDeadband(joystickLeft.getX(), DEADBAND),
                MAX_VELOCITY * Utils.checkDeadband(-joystickLeft.getY(), DEADBAND),
                MAX_ROTATIONAL_VELOCITY * Utils.checkDeadband(joystickRight.getX(), DEADBAND)
        );
    }

    /**
     * Terminate all movement in the swerve drive.
     *
     * @param interrupted whether the command is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
