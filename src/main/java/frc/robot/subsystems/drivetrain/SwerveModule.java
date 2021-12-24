package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;

public class SwerveModule extends SubsystemBase {
    private int i;
    private final TalonSRX motor = new TalonSRX(Constants.Swerve.MOTOR);
    private final TalonFX driveMotor = new TalonFX(Constants.Swerve.MOTOR);
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Swerve.VELOCITY_TICKS_PER_UNIT);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Swerve.DEGREE_TICKS_PER_UNIT);

    public SwerveModule(int i) {
        this.i = i;
    }

    public double getAngle() {
        return unitModelDegree.toUnits(motor.getSelectedSensorPosition());
    }

    public double getTargetError(double angle, double currentAngle){
        return getTargetError(angle - currentAngle);
    }

    public void setAngle(double angle) {
        angle = Math.IEEEremainder(angle, 360);
        motor.set(ControlMode.Position, unitModelDegree.toTicks(angle));
    }

    public double getVelocity() {
        return unitModelVelocity.toVelocity(motor.getSelectedSensorVelocity());
    }

    public void setVelocity(double velocity) {
        driveMotor.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
    }

    public void state(double getVelocity, double getAngle) {
        setAngle(getAngle);
        setVelocity(getVelocity);
    }

    public void stop() {
        setVelocity(0);
        setAngle(getAngle());
    }

    public void maxPower() {
       driveMotor.set(ControlMode.PercentOutput, 1);
    }

}
