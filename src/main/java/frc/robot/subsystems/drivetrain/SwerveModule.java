package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

/**
 * Add angle motor, drive motor, unitModelVelocity and unitModel.
 */
public class SwerveModule extends SubsystemBase {
    private final WPI_TalonSRX angleMotor;
    private final WPI_TalonFX driveMotor;
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Swerve.VELOCITY_TICKS_PER_DEGREE);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Swerve.DEGREE_TICKS_PER_DEGREE);
    private final int i;

    public SwerveModule(int i) {
        this.i = i;

        /**
         * Add PID, encoder, Voltage Compensation, Current Limit.
         */
        this.angleMotor = new WPI_TalonSRX(Ports.ExampleSubsystem.ANGLE_MOTOR[i]);
        angleMotor.config_kP(0, Constants.Swerve.ANGLE_MOTOR_P[i]);
        angleMotor.config_kI(0, Constants.Swerve.ANGLE_MOTOR_I[i]);
        angleMotor.config_kD(0, Constants.Swerve.ANGLE_MOTOR_D[i]);
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configPeakCurrentLimit(Constants.Swerve.CURRENT_LIMIT, 10);
        angleMotor.configVoltageCompSaturation(0,10);

        /**
         * Add PID, encoder, Closed loop Ramp, Voltage Compensation, CurrentLimit.
         */
        this.driveMotor = new WPI_TalonFX(Ports.ExampleSubsystem.DRIVE_MOTOR[i]);
        driveMotor.config_kP(0, Constants.Swerve.ANGLE_MOTOR_P[i]);
        driveMotor.config_kI(0, Constants.Swerve.ANGLE_MOTOR_I[i]);
        driveMotor.config_kD(0, Constants.Swerve.ANGLE_MOTOR_D[i]);
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.configClosedloopRamp(Constants.Swerve.RAMP_RATE);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.Swerve.CURRENT_LIMIT, Constants.Swerve.TRIGGER_CURRENT, Constants.Swerve.TRIGGER_TIME), 10);
        angleMotor.configVoltageCompSaturation(0,10);
    }

    /**
     * Find the smallest error.
     * @param angle
     * @param currentAngle
     * @return
     */
    public static double getTargetError(double angle, double currentAngle) {
        angle = Math.IEEEremainder(angle, 360);
        currentAngle =  Math.IEEEremainder(currentAngle, 360);

        if (angle >= 0 && currentAngle >= 0) {
            return angle - currentAngle;
        }

        else if (angle > 0 && currentAngle < 0) {
            if (angle + Math.abs(currentAngle) <= 180){
                return angle + Math.abs(currentAngle);
            }
            else {
                return (angle - (360 + currentAngle));
            }
        }

        else if (angle < 0 && currentAngle > 0) {
            if (Math.abs(angle) + currentAngle <= 180) {
                return (Math.abs(angle) + currentAngle)* -1 ;
            } else {
                return currentAngle + angle;
            }
        }

        else if (angle <= 0 && currentAngle <= 0) {
            return Math.abs(currentAngle) + angle;
        }
        return 0;
    }

    /**
     * get angle
     * @return angle
     */

    public double getAngle() {
        return Math.IEEEremainder(unitModelDegree.toUnits(angleMotor.getSelectedSensorPosition()), 360);
    }

    /**
     * Set angle.
     * @param angle the desired angle.
     */

    public void setAngle(double angle) {
        angle = Math.IEEEremainder(angle, 360);
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition() + unitModelDegree.toTicks(getTargetError(angle, getAngle())));
    }

    /**
     * Get velocity.
     * @return sensor units per 100ms.
     */
    public double getVelocity() {
        return unitModelVelocity.toVelocity(driveMotor.getSelectedSensorVelocity());
    }

    /**
     * Set velocity.
     */
    public void setVelocity(double velocity) {
        driveMotor.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
    }

    public void state(double velocity, double angle) {
        setAngle(angle);
        setVelocity(velocity);
    }

    /**
     * Pose angle, stop power.
     */
    public void stop() {
        setVelocity(0);
        setAngle(getAngle());
    }

    /**
     * Power out of 1.
     */

    public void maxPower() {
        driveMotor.set(ControlMode.PercentOutput, 1);
        angleMotor.set(ControlMode.PercentOutput, 1);
    }
}