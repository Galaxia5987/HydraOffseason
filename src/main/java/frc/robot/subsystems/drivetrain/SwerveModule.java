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
import frc.robot.subsystems.UnitModel;

public class SwerveModule extends SubsystemBase {
    private WPI_TalonSRX angleMotor;
    private WPI_TalonFX driveMotor;
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Swerve.VELOCITY_TICKS_PER_UNIT);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Swerve.DEGREE_TICKS_PER_UNIT);
    private  int i;

    public SwerveModule(int i) {
        this.i = i;

        this.angleMotor = new WPI_TalonSRX(Constants.Swerve.ANGLE_MOTOR[i]);
        angleMotor.config_kP(0, Constants.Swerve.ANGLE_MOTOR_PID_P[i]);
        angleMotor.config_kI(0, Constants.Swerve.ANGLE_MOTOR_PID_I[i]);
        angleMotor.config_kD(0, Constants.Swerve.ANGLE_MOTOR_PID_D[i]);
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configPeakCurrentLimit(Constants.Swerve.CURRENT_LIMIT, 10);

        this.driveMotor = new WPI_TalonFX(Constants.Swerve.DRIVE_MOTOR[i]);
        driveMotor.config_kP(0, Constants.Swerve.ANGLE_MOTOR_PID_P[i]);
        driveMotor.config_kI(0, Constants.Swerve.ANGLE_MOTOR_PID_I[i]);
        driveMotor.config_kD(0, Constants.Swerve.ANGLE_MOTOR_PID_D[i]);
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.configClosedloopRamp(Constants.Swerve.RAMP_RATE);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.Swerve.CURRENT_LIMIT, Constants.Swerve.TRIGGER_CURRENT, Constants.Swerve.TRIGGER_TIME), 10);
    }




    public static double getTargetError(double angle, double currentAngle) {
        angle = Math.IEEEremainder(angle, 360);
        currentAngle = Math.IEEEremainder(currentAngle, 360);

        if (angle >= 0 && currentAngle >= 0) {
            return angle - currentAngle;
        } else if (angle > 0 && currentAngle < 0) {
            if (angle + Math.abs(currentAngle) <= 180) {
                return angle + Math.abs(currentAngle);
            } else if (Math.abs(currentAngle) + angle >= 180) {
                return (angle + currentAngle) * -1;
            } else {
                return Math.abs(currentAngle) + angle;
            }
        } else if (angle < 0 && currentAngle > 0) {
            if (Math.abs(angle) + currentAngle <= 180) {
                return (Math.abs(angle) + currentAngle) * -1;
            } else {
                return currentAngle + angle;
            }
        } else if (angle <= 0 && currentAngle <= 0) {
            return Math.abs(currentAngle) + angle;
        }
        return 0;
    }

    public double getAngle() {
        return Math.IEEEremainder(unitModelDegree.toUnits(angleMotor.getSelectedSensorPosition()), 360);
    }

    public void setAngle(double angle) {
        angle = Math.IEEEremainder(angle, 360);
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition() + unitModelDegree.toTicks(getTargetError(angle, getAngle())));
    }

    public double getVelocity() {
        return unitModelVelocity.toVelocity(angleMotor.getSelectedSensorVelocity());
    }

    public void setVelocity(double velocity) {
        driveMotor.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
    }

    public void state(double velocity, double angle) {
        setAngle(angle);
        setVelocity(velocity);
    }

    public void stop() {
        setVelocity(0);
        setAngle(getAngle());
    }

    public void maxPower() {
        driveMotor.set(ControlMode.PercentOutput, 1);
        angleMotor.set(ControlMode.PercentOutput, 1);
    }

    public void setRightPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setLeftPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

}