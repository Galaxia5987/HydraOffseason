package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitModel;

public class SwerveModule {
    //    private final WPI_TalonFX driveMotor = new WPI_TalonFX(0);
//    private final WPI_TalonSRX angleMotor = new WPI_TalonSRX(0);
    private final UnitModel unitModel = new UnitModel(Constants.ExampleSubsystem.TICKS_TO_DEGREAS);
    private final int i;

    public SwerveModule(int i) {
        this.i = i;
    }


    /*
        public double getVelocity() {
            return driveMotor.getSelectedSensorVelocity();
        }

        public void setVelocity(double velocity) {
            driveMotor.set(ControlMode.Velocity, velocity);
        }

        public double getAngle() {
            return unitModel.toUnits((angleMotor.getSelectedSensorPosition() - Constants.ExampleSubsystem.INITIAL_TICKS[0]) % Constants.ExampleSubsystem.TICKS_PER_CIRCLE);
        }

        public void setAngle(double Angle) {
            angleMotor.set(ControlMode.Position, angleMotor.get() + unitModel.toTicks(( Angle - getAngle())));
        }
    */
    public double getclosest(double requiredAngle, double currentAngle) {
        if (Math.abs(requiredAngle - currentAngle > 180)) {
            return (360) - ((requiredAngle - currentAngle));
        } else if (requiredAngle > 180 && currentAngle < 180 || requiredAngle < 180 && currentAngle > 180){
            return currentAngle - requiredAngle;
        }
        else {
            return (currentAngle) + ((requiredAngle - currentAngle));
        }
    }
}


