package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public final class Ports {
    public static final int XBOX_CONTROLLER_PORT = 2;

    public static final class SwervePorts {
        public static final int FR_DRIVE_MOTOR_PORT = 27;
        public static final int FL_DRIVE_MOTOR_PORT = 23;
        public static final int RR_DRIVE_MOTOR_PORT = 25;
        public static final int RL_DRIVE_MOTOR_PORT = 21;

        public static final int FR_ANGLE_MOTOR_PORT = 28;
        public static final int FL_ANGLE_MOTOR_PORT = 24;
        public static final int RR_ANGLE_MOTOR_PORT = 26;
        public static final int RL_ANGLE_MOTOR_PORT = 22;

        public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

        private static final boolean[] FR_INVERSIONS = {false, false, false, false};
        private static final boolean[] FL_INVERSIONS = {true, false, true, false};
        private static final boolean[] RR_INVERSIONS = {false, false, false, false};
        private static final boolean[] RL_INVERSIONS = {false, false, false, true};
        public static final boolean[][] INVERSIONS = {FR_INVERSIONS, FL_INVERSIONS, RR_INVERSIONS, RL_INVERSIONS};

        public static final boolean NOT_CONTINUOUS_FEEDBACK = true;
    }

    public static final class PhotonVisionPorts {
        public static final int PISTON_PORT = 4;
    }

    public static final class ShooterPorts {
        public static final int SHOOTER_MOTOR_PORT = 32;
        public static final int SHOOTER_SLAVE_PORT = 33;
        public static final int HOOD_MOTOR_PORT = 34;

        public static final boolean SHOOTER_MOTOR_INVERTED = true;
        public static final boolean SHOOTER_SLAVE_INVERTED = true;
        public static final boolean SHOOTER_SENSOR_INVERTED = false;

        public static final boolean HOOD_INVERTED = true;
        public static final boolean HOOD_SENSOR_INVERTED = true;

    }

    public static final class ConveyorPorts {
        public static final int MOTOR_PORT = 41;

        public static final boolean MOTOR_INVERTED = true;
    }

    public static final class FunnelPorts {
        public static final int MOTOR_PORT = 11;
        public static final int PISTON_PORT = 6;

        public static final boolean MOTOR_INVERTED = false;
    }

    public static final class IntakePorts {
        public static final int MOTOR_PORT = 18;
        public static final int PISTON_PORT = 5;

        public static final boolean MOTOR_INVERTED = true;
    }
}
