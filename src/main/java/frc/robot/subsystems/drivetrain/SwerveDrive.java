package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

import static frc.robot.Constants.SwerveDrive.*;

public class SwerveDrive extends SubsystemBase {
    private Timer timer = new Timer();
    private double currentTime;
    private double lastTime = 0;
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Ry, Rx),
            new Translation2d(Ry, -Rx),
            new Translation2d(-Ry, Rx),
            new Translation2d(-Ry, -Rx)
    );
    private final SwerveModuleState[] states = new SwerveModuleState[4];
    private boolean fieldOriented;

    /**
     * Constructor.
     *
     * @param fieldOriented whether the robot is field oriented.
     */
    public SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;

        timer.start();
        configModules();
    }

    /**
     * Sets the states for the swerve drive.
     *
     * @param vx       is the velocity in the x direction. [m/s]
     * @param vy       is the velocity in the y direction. [m/s]
     * @param rotation is the rotation velocity. [rad/s]
     */
    public void setStates(double vx, double vy, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx, vy, rotation, Rotation2d.fromDegrees(Robot.navx.getYaw())
                ) :
                new ChassisSpeeds(vx, vy, rotation);

        for (int i = 0; i < 4; i++) {
            states[i] = kinematics.toSwerveModuleStates(speeds)[i];
            SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(Robot.navx.getYaw()));
        }

        SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY);

        for (int i = 0; i < 4; i++) {
            modules[i].setAngle(states[i].angle.getDegrees());
            modules[i].setVelocity(states[i].speedMetersPerSecond, currentTime - lastTime);
        }

        lastTime = currentTime;
    }

    /**
     * Sets whether the robot is field oriented.
     *
     * @param fieldOriented is whether the robot is field oriented.
     */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    /**
     * Configures all the settings of each module.
     */
    public void configModules() {
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule();
            modules[i].setMotorPorts(Ports.SwerveDrive.motorPorts[i]);
            modules[i].setAnglePID(Constants.SwerveDrive.anglePID[i]);
            modules[i].setDrivePID(Constants.SwerveDrive.drivePID[i]);
            modules[i].configInverted(Ports.SwerveDrive.motorINVERTED[i]);
            modules[i].configSensorPhase(Ports.SwerveDrive.motorSENSOR_PHASE[i]);
        }
    }

    /**
     * Checks whether a value is in a specified dead band.
     *
     * @param val      is the value.
     * @param deadBand is the dead band that the function checks.
     * @return 0 or the input val (depending on the dead band).
     */
    public double checkDeadBand(double val, double deadBand) {
        return (Math.abs(val) < deadBand) ? 0 : val;
    }

    /**
     * Terminates all movement of the swerve drive.
     */
    public void terminate() {
        for (int i = 0; i < 4; i++) {
            modules[i].terminate();
        }
    }

    @Override
    public void periodic() {
        currentTime = timer.get();
    }
}
