// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public static final double SKEW_X_FR_METERS = 0.75;
        public static final double SKEW_Y_FR_METERS = 0.75;
        public static final double SKEW_X_FL_METERS = -0.75;
        public static final double SKEW_Y_FL_METERS = 0.75;
        public static final double SKEW_X_RR_METERS = 0.75;
        public static final double SKEW_Y_RR_METERS = -0.75;
        public static final double SKEW_X_RL_METERS = -0.75;
        public static final double SKEW_Y_RL_METERS = -0.75;

        public static final double TICKS_PER_METER = 48122.439486053394;
        public static final double TICKS_PER_DEGREE = 0.9266354464461463;

        public static final double DRIVE_KP = 0.2;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0;

        public static final double DRIVE_SPEED_MULTIPLIER = 2 / Math.sqrt(2);
        public static final double ROTATION_SPEED_MULTIPLIER = 2 * Math.PI;

        public static final double ANGLE_TOLERANCE = 1;
    }

    public static final class PhotonVisionConstants {
        public static final double CAMERA_HEIGHT = 0.66;
        public static final double TARGET_HEIGHT = 2.5;
        public static final double LONG_RANGE_CAMERA_PITCH = 34;
        public static final double SHORT_RANGE_CAMERA_PITCH = 56;
        public static final int SHORT_RANGE_PIPELINE_INDEX = 0; //TODO: verify.
        public static final int LONG_RANGE_PIPELINE_INDEX = 1; //TODO: verify.
    }

    public static final class ShooterConstants {
        public static final double SHOOTER_KP = 0.2;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;

        public static final double HOOD_KP = 0.2;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;

        public static final int TICKS_PER_REVOLUTION = 2048;

        public static final int HOOD_MAX_POSITION = 5634;
        public static final double G = 9.80665;
        public static final double TARGET_HEIGHT = 2.5;
        public static final double SHOOTER_HEIGHT = 0.66;
        public static final double FLYWHEEL_RADIUS = 0.05;
        public static final double FLYWHEEL_SPEED_MULTIPLIER = 3;

        public static final double OUTTAKE_POWER = 0.5;

        public static final double VELOCITY_TOLERANCE = 5;
        public static final double HOOD_TICKS_TOLERANCE = 50;
    }

    public static final class ConveyorConstants {
        public static final double CONVEYOR_POWER = 0.5;
    }

    public static final class FunnelConstants {
        public static final double FUNNEL_POWER = 0.5;
        public static final double PISTON_PERIOD = 3;
    }

    public static final class IntakeConstants {
        public static final double INTAKE_POWER = 0.5;
    }


}
