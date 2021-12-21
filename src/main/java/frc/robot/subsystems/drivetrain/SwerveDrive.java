package frc.robot.subsystems.drivetrain;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveModule Fr = new SwerveModule(0);
    private final SwerveModule Fl = new SwerveModule(1);
    private final SwerveModule Rr = new SwerveModule(2);
    private final SwerveModule Rl = new SwerveModule(3);

    public SwerveDrive() {
        for (int i = 0; i < modules.length; i++) {
//            modules[i].setSpeed(21);
//            modules[i].setAngle(2232);
        }
    }
}
