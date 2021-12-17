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
        private static final int[] fr_ports = new int[]{0, 0}; // ports for the front right motors. [drive, angle]
        private static final int[] fl_ports = new int[]{0, 0}; // ports for the front left motors. [drive, angle]
        private static final int[] rr_ports = new int[]{0, 0}; // ports for the rear right motors. [drive, angle]
        private static final int[] rl_ports = new int[]{0, 0}; // ports for the rear left motors. [drive, angle]

        public static final int[][] motorPorts = new int[][]{ // all the ports for the motors.
                fr_ports, fl_ports, rr_ports, rl_ports
        };

        private static final boolean[] fr_INVERTED = new boolean[]{false, false}; // inversions for the front right motors. [drive, angle]
        private static final boolean[] fl_INVERTED = new boolean[]{false, false}; // inversions for the front left motors. [drive, angle]
        private static final boolean[] rr_INVERTED = new boolean[]{false, false}; // inversions for the rear right motors. [drive, angle]
        private static final boolean[] rl_INVERTED = new boolean[]{false, false}; // inversions for the rear left motors. [drive, angle]

        public static final boolean[][] motorINVERTED = new boolean[][]{ // all the inversions for the motors.
                fr_INVERTED, fl_INVERTED, rr_INVERTED, rl_INVERTED
        };
    }
}
