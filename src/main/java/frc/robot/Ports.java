package frc.robot;

public final class Ports {
    public static class ExampleSubsystem {
        public static final int MAIN = 0;
        public static final int AUX = 0;
        public static final boolean IS_MAIN_INVERTED = false;
        public static final boolean IS_AUX_INVERTED = false;
        public static final boolean IS_MAIN_SENSOR_INVERTED = false;
        public static final boolean IS_AUX_SENSOR_INVERTED = false;
    }

    public static class Controls {
        public static final int XBOX = 0; // Xbox controller port.
        public static final int JOYSTICK_RIGHT = 0; // Right joystick port.
        public static final int JOYSTICK_LEFT = 0; // Left joystick port.
    }

    public static class SwerveDrive {
        private static final int[] fr_Ports = new int[]{0, 0}; // ports for the front right motors. [drive, angle]
        private static final int[] fl_Ports = new int[]{0, 0}; // ports for the front left motors. [drive, angle]
        private static final int[] rr_Ports = new int[]{0, 0}; // ports for the rear right motors. [drive, angle]
        private static final int[] rl_Ports = new int[]{0, 0}; // ports for the rear left motors. [drive, angle]

        public static final int[][] motorPorts = new int[][]{ // all the ports for the motors.
                fl_Ports, fr_Ports, rl_Ports, rr_Ports
        };

        private static final boolean[] fr_INVERTED = new boolean[]{false, false}; // inversions for the front right motors. [drive, angle]
        private static final boolean[] fl_INVERTED = new boolean[]{false, false}; // inversions for the front left motors. [drive, angle]
        private static final boolean[] rr_INVERTED = new boolean[]{false, false}; // inversions for the rear right motors. [drive, angle]
        private static final boolean[] rl_INVERTED = new boolean[]{false, false}; // inversions for the rear left motors. [drive, angle]

        public static final boolean[][] motor_INVERTED = new boolean[][]{ // all the inversions for the motors.
                fl_INVERTED, fr_INVERTED, rl_INVERTED, rr_INVERTED
        };

        private static final boolean[] fr_SENSOR_PHASE = new boolean[]{false, false}; // inversions for the front right motors. [drive, angle]
        private static final boolean[] fl_SENSOR_PHASE = new boolean[]{false, false}; // inversions for the front left motors. [drive, angle]
        private static final boolean[] rr_SENSOR_PHASE = new boolean[]{false, false}; // inversions for the rear right motors. [drive, angle]
        private static final boolean[] rl_SENSOR_PHASE = new boolean[]{false, false}; // inversions for the rear left motors. [drive, angle]

        public static final boolean[][] motor_SENSOR_PHASE = new boolean[][]{ // all the inversions for the motors.
                fl_SENSOR_PHASE, fr_SENSOR_PHASE, rl_SENSOR_PHASE, rr_SENSOR_PHASE
        };
    }
}
