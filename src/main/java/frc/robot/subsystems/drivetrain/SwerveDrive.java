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
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Rx, Ry),
            new Translation2d(-Rx, Ry),
            new Translation2d(Rx, -Ry),
            new Translation2d(-Rx, -Ry)
    );
    private final Timer timer = new Timer();
    private double currentTime;
    private double lastTime = 0;
    private boolean fieldOriented;

    private static SwerveDrive INSTANCE = null;

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

    public static SwerveDrive getInstance(){
        if(INSTANCE == null)
            INSTANCE = new SwerveDrive(true);
        return INSTANCE;
    }

    /**
     * Sets the states for the swerve drive.
     *
     * @param vx       is the velocity in the x direction. [m/s]
     * @param vy       is the velocity in the y direction. [m/s]
     * @param rotation is the rotation velocity. [rad/s]
     */
    public void holonomicDrive(double vx, double vy, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx, vy, rotation, Rotation2d.fromDegrees(Robot.navx.getYaw())
                ) :
                new ChassisSpeeds(vx, vy, rotation);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(Robot.navx.getYaw()));

            modules[i].setAngle(states[i].angle.getDegrees());
            modules[i].setVelocity(states[i].speedMetersPerSecond, currentTime - lastTime);
        }
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
            modules[i].configInverted(Ports.SwerveDrive.motor_INVERTED[i]);
            modules[i].configSensorPhase(Ports.SwerveDrive.motor_SENSOR_PHASE[i]);
        }
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
        lastTime = currentTime;

        currentTime = timer.get();
    }
}
