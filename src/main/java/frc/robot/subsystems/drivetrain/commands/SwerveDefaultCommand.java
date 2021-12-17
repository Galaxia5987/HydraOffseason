package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.SwerveDrive.DEAD_BAND;
import static frc.robot.Ports.Controls.JOYSTICK_LEFT;
import static frc.robot.Ports.Controls.JOYSTICK_RIGHT;

public class SwerveDefaultCommand extends CommandBase {
    private final Joystick joystickRight = new Joystick(JOYSTICK_RIGHT);
    private final Joystick joystickLeft = new Joystick(JOYSTICK_LEFT);
    private final boolean isFieldOriented;
    private SwerveDrive swerveDrive;

    /**
     * Constructor.
     *
     * @param isFieldOriented whether the robot is field oriented.
     */
    public SwerveDefaultCommand(boolean isFieldOriented, SwerveDrive swerveDrive) {
        this.isFieldOriented = isFieldOriented;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        swerveDrive.setFieldOriented(isFieldOriented);
    }

    /**
     * Sets the states in a loop according to the joystick double suppliers.
     */
    @Override
    public void execute() {
        swerveDrive.setStates(swerveDrive.checkDeadBand(joystickLeft.getX(), DEAD_BAND),
                swerveDrive.checkDeadBand(-joystickLeft.getY(), DEAD_BAND),
                swerveDrive.checkDeadBand(joystickRight.getX(), DEAD_BAND)
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
