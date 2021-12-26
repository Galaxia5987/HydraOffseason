package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;

public class SwerveModule extends SubsystemBase {
    private final TalonSRX angleMotor = new TalonSRX(Constants.Swerve.MOTOR);
    private final TalonFX driveMotor = new TalonFX(Constants.Swerve.MOTOR);
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Swerve.VELOCITY_TICKS_PER_UNIT);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Swerve.DEGREE_TICKS_PER_UNIT);
    private final int i;

    public SwerveModule(int i) {
        this.i = i;
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
}