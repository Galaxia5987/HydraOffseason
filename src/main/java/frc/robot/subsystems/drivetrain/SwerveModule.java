package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.SwerveDrive.*;

public class SwerveModule extends SubsystemBase {
    private final UnitModel angle_unitModel = new UnitModel(TICKS_PER_RADIAN);
    private final UnitModel drive_unitModel = new UnitModel(TICKS_PER_METER);
    private WPI_TalonFX driveMotor;
    private WPI_TalonSRX angleMotor;

    /**
     * Constructor.
     */
    public SwerveModule() {
    }

    /**
     * Sets the angle PID controller for the specific module.
     *
     * @param anglePID is the array of PID configurations.
     */
    public void setAnglePID(double[] anglePID) {
        angleMotor.config_kP(0, anglePID[0]);
        angleMotor.config_kI(0, anglePID[1]);
        angleMotor.config_kD(0, anglePID[2]);
    }

    /**
     * Sets the drive PID controller for the specific module.
     *
     * @param drivePID is the array of PID configurations.
     */
    public void setDrivePID(double[] drivePID) {
        driveMotor.config_kP(0, drivePID[0]);
        driveMotor.config_kI(0, drivePID[1]);
        driveMotor.config_kD(0, drivePID[2]);
    }

    /**
     * Sets the motor ports for the specific module.
     *
     * @param ports is the array of ports.
     */
    public void setMotorPorts(int[] ports) {
        driveMotor = new WPI_TalonFX(ports[0]);
        angleMotor = new WPI_TalonSRX(ports[1]);
    }

    /**
     * Configures the inversions for the motors.
     *
     * @param inverted is the array of inversions.
     */
    public void configInverted(boolean[] inverted) {
        driveMotor.setInverted(inverted[0]);
        angleMotor.setInverted(inverted[1]);
    }

    /**
     * Gets the current angle of the module.
     *
     * @return the angle of the module. [rad]
     */
    public double getAngle() {
        return Math.IEEEremainder(
                angle_unitModel.toUnits(angleMotor.getSelectedSensorPosition()),
                Math.PI * 2
        );
    }

    /**
     * Sets the angle for the module.
     *
     * @param angle is the angle to set the module to. [rad]
     */
    public void setAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle_unitModel.toTicks(angle));
    }

    /**
     * Gets the velocity of the module.
     *
     * @return the velocity of the module. [m/s]
     */
    public double getVelocity() {
        return drive_unitModel.toVelocity(driveMotor.getSelectedSensorVelocity());
    }

    /**
     * Sets the velocity of the module.
     *
     * @param velocity is the velocity to set the module to. [m/s]
     */
    public void setVelocity(double velocity) {
        driveMotor.set(ControlMode.Velocity, drive_unitModel.toTicks100ms(velocity));
    }

    /**
     * Sets all the speeds of the module to 0.
     */
    public void terminate() {
        angleMotor.set(ControlMode.PercentOutput, 0);
        driveMotor.set(ControlMode.PercentOutput, 0);
    }
}
