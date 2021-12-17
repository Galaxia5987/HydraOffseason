package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Swerve.Rx;
import static frc.robot.Constants.Swerve.Ry;
import frc.robot.Constants;

import java.lang.invoke.ConstantBootstraps;

public class SwerveDrive extends SubsystemBase {
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics
            (new Translation2d(Rx, Ry),
                    new Translation2d(-Rx, Ry),
                    new Translation2d(Rx, -Ry),
                    new Translation2d(-Rx, -Ry)
            );

    public stop{

    }

    public getModule{

    }

    public drive{

    }

    public setStates{

    }

    public static void main(String[] args) {
        for (int i = 0; i<4; i++){

        }
    }
}

