package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

import static frc.robot.Constants.NOMINAL_VOLTAGE;
import static frc.robot.Constants.SwerveDrive.TICKS_PER_DEGREE_ANGLE;
import static frc.robot.Constants.SwerveDrive.TICKS_PER_METER_DRIVE;
import static frc.robot.Constants.SwerveModule.*;

public class SwerveModule extends SubsystemBase {
    private final UnitModel angle_unitModel = new UnitModel(TICKS_PER_DEGREE_ANGLE);
    private final UnitModel drive_unitModel = new UnitModel(TICKS_PER_METER_DRIVE);
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private WPI_TalonFX driveMotor;
    private WPI_TalonSRX angleMotor;

    /**
     * Constructor.
     */
    public SwerveModule() {
        LinearSystem<N1, N1, N1> module = new LinearSystem<>(
                A, B, C, D
        );
        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter(
                Nat.N1(),
                Nat.N1(),
                module,
                MODEL_TOLERANCE,
                SENSOR_TOLERANCE,
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> quadraticRegulator = new LinearQuadraticRegulator<>(
                module,
                VecBuilder.fill(MODEL_TOLERANCE.get(0, 0)),
                VecBuilder.fill(SENSOR_TOLERANCE.get(0, 0)),
                Constants.LOOP_PERIOD
        );
        linearSystemLoop = new LinearSystemLoop<>(
                module, quadraticRegulator, kalmanFilter, NOMINAL_VOLTAGE, Constants.LOOP_PERIOD
        );
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
     * Configure sensor phase for module.
     *
     * @param sensorPhase is the array of sensor phases.
     */
    public void configSensorPhase(boolean[] sensorPhase) {
        driveMotor.setSensorPhase(sensorPhase[0]);
        angleMotor.setSensorPhase(sensorPhase[1]);
    }

    public void setZeroPosition(int[] positions){
        driveMotor.setSelectedSensorPosition(positions[0]);
        angleMotor.setSelectedSensorPosition(positions[1]);
    }

    /**
     * Sets the postions of the encoders.
     *
     * @param positions is the array of positions.
     */
    public void setZeroPosition(int[] positions) {
        driveMotor.setSelectedSensorPosition(positions[0]);
        angleMotor.setSelectedSensorPosition(positions[1]);
    }

    /**
     * Gets the current angle of the module.
     *
     * @return the angle of the module. [deg]
     */
    public double getAngle() {
        return Math.IEEEremainder(
                angle_unitModel.toUnits(angleMotor.getSelectedSensorPosition()),
                360
        );
    }

    /**
     * Sets the angle for the module.
     *
     * @param reqAngle is the angle to set the module to. [deg]
     */
    public void setAngle(double reqAngle) {
        double currAngle = getAngle();

        currAngle %= 360;
        currAngle = (currAngle < 0) ? (360 + currAngle) : currAngle;

        reqAngle %= 360;
        reqAngle = (reqAngle < 0) ? (360 + reqAngle) : reqAngle;

        double error = reqAngle - currAngle;
        error = (Math.abs(error) > 180) ? (error - (360 * Math.signum(error))) : error;
        error %= 360;
        error = Utils.checkDeadband(error, 0.01);

        angleMotor.set(
                ControlMode.Position,
                angleMotor.getSelectedSensorPosition() + angle_unitModel.toTicks(error)
        );
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
    public void setVelocity(double velocity, double timeInterval) {
        velocity = Utils.checkDeadband(velocity, 0.1);

        linearSystemLoop.setNextR(VecBuilder.fill(velocity));
        linearSystemLoop.correct(VecBuilder.fill(getVelocity()));
        linearSystemLoop.predict(timeInterval);

        driveMotor.set(ControlMode.PercentOutput, linearSystemLoop.getU(0) / NOMINAL_VOLTAGE);
    }

    /**
     * Sets all the speeds of the module to 0.
     */
    public void terminate() {
        angleMotor.set(ControlMode.PercentOutput, 0);
        driveMotor.set(ControlMode.PercentOutput, 0);
    }
}
