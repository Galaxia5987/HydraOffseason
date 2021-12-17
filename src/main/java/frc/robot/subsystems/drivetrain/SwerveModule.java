package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;

import java.awt.image.TileObserver;

public class SwerveModule extends SubsystemBase {
    private final TalonSRX motor = new TalonSRX(Constants.Swerve.MOTOR);
    private final TalonFX driveMotor = new TalonFX(Constants.Swerve.MOTOR);
    private final UnitModel unitModel = new UnitModel(Constants.Swerve.TICKS_PER_UNIT);

    public double setAngle(double angle) {
        motor.set(ControlMode.Position, angle );
    }

    public double getAngle{
        return ;
    }


    public double setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity);
    }

    public double getVelocity(){
        return unitModel().toVelocity(motor.getSelectedSensorVelocity());
    }

    public double state(double getVelocity, double getAngle) {

    }

    public void stop(){

    }

    public void maxPower(){

    }
}
