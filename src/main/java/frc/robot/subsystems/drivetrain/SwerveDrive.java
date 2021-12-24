package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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

    public SwerveModule getModule(int i) {
        return swerveModules[i];
    }

    public void drive(double forward, double strafe, double rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(Robot.navx.getAngle()));
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i<4; i++){
            getModule(i).state(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }
}


