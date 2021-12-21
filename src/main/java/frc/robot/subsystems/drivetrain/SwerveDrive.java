package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.Rx;
import static frc.robot.Constants.Swerve.Ry;

public class SwerveDrive extends SubsystemBase {
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics
            (new Translation2d(Rx, Ry),
                    new Translation2d(-Rx, Ry),
                    new Translation2d(Rx, -Ry),
                    new Translation2d(-Rx, -Ry)
            );
    private final SwerveModule[] swerveModules;

    public SwerveDrive(SwerveModule[] swerveModule) {
        this.swerveModules = swerveModule;
    }

    public void getModule() {

    }

    public void drive() {

    }

    public void setStates() {

    }

}


