package frc.robot;


import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.opencv.core.Mat;

public final class Constants {
    public static final double WHEEL_RADIUS = 1;
    public static final double LOOP_PERIOD = 0.02;

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class SwerveDrive {
        public static final double[] fr_anglePID = new double[]{0, 0, 0}; // front right PID controller for angle.
        public static final double[] fl_anglePID = new double[]{0, 0, 0}; // front left PID controller for angle.
        public static final double[] rr_anglePID = new double[]{0, 0, 0}; // rear right PID controller for angle.
        public static final double[] rl_anglePID = new double[]{0, 0, 0}; // rear left PID controller for angle.

        public static final double[][] anglePID = new double[][]{ // all the PID configurations for angle.
                fl_anglePID, fr_anglePID, rl_anglePID, rr_anglePID
        };

        public static final double[] fr_drivePID = new double[]{0, 0, 0}; // front right PID controller for drive.
        public static final double[] fl_drivePID = new double[]{0, 0, 0}; // front left PID controller for drive.
        public static final double[] rr_drivePID = new double[]{0, 0, 0}; // rear right PID controller for drive.
        public static final double[] rl_drivePID = new double[]{0, 0, 0}; // rear left PID controller for drive.

        public static final double[][] drivePID = new double[][]{ // all the PID configurations for drive.
                fl_drivePID, fr_drivePID, rl_drivePID, rr_drivePID
        };

        public static final double TICKS = 1024;
        public static final double TICKS_PER_RADIAN = TICKS / (2 * Math.PI); // ticks per radian. [ticks/rad]
        public static final double TICKS_PER_METER = TICKS / (2 * Math.PI * WHEEL_RADIUS); // ticks per meter. [ticks/m]

        public static final double Rx = 0.75 / 2; // x position of the modules. [m]
        public static final double Ry = 0.75 / 2; // y position of the modules. [m]

        public static final double MAX_VELOCITY = 4.5; // maximum velocity of the modules. [m/s]

        public static final double DEAD_BAND = 0.02; // dead band for the joysticks.
    }

    public static class SwerveModule{
        public static final double NOMINAL_VOLTAGE = 12.0;
        public static final double Kv = 1;
        public static final double Ka = 1;
        public static final Matrix<N1, N1> A = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                -Kv / Ka
        );
        public static final Matrix<N1, N1> B = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1 / Ka
        );
        public static final Matrix<N1, N1> C = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1
        );
        public static final Matrix<N1, N1> D = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                0
        );

        public static final Matrix<N1, N1> MODEL_TOLERANCE = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                0.8
        );
        public static final Matrix<N1, N1> SENSOR_TOLERANCE = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                0.2
        );
    }
}
