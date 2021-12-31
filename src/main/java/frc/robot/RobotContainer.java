package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.SwerveDefaultCommand;
import frc.robot.subsystems.example.ExampleSubsystem;

public class RobotContainer {
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    // The robot's subsystems and commands are defined here...
    public ExampleSubsystem exampleSubsystem = ExampleSubsystem.getInstance();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(
                new SwerveDefaultCommand(swerveDrive)
        );
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        return null;
    }
}
