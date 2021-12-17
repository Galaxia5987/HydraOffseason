package frc.robot;


public final class Constants {
    public static final double WHEEL_RADIUS = 1;

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class SwerveDrive {
        public static final double[] fr_anglePID = new double[] {0, 0, 0}; // front right PID controller for angle.
        public static final double[] fl_anglePID = new double[] {0, 0, 0}; // front left PID controller for angle.
        public static final double[] rr_anglePID = new double[] {0, 0, 0}; // rear right PID controller for angle.
        public static final double[] rl_anglePID = new double[] {0, 0, 0}; // rear left PID controller for angle.

        public static final double[][] anglePID = new double[][]{ // all the PID configurations for angle.
                fl_anglePID, fr_anglePID, rl_anglePID, rr_anglePID
        };

        public static final double[] fr_drivePID = new double[] {0, 0, 0}; // front right PID controller for drive.
        public static final double[] fl_drivePID = new double[] {0, 0, 0}; // front left PID controller for drive.
        public static final double[] rr_drivePID = new double[] {0, 0, 0}; // rear right PID controller for drive.
        public static final double[] rl_drivePID = new double[] {0, 0, 0}; // rear left PID controller for drive.

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

}
