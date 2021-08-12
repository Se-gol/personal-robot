package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilEverything;

import java.util.Arrays;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Ports.SwervePorts.*;

/**
 * This subsystem controls all 4 modules of the robot.
 * All angle units are in degrees and are in accordance with the unit circle (https://i.stack.imgur.com/5zOW8.gif).
 * Default arrangement for arrays is front-right, front-left, rear-right, rear-left.
 * x right is positive, y up is positive, cw is positive.
 */
public class SwerveSubsystem extends SubsystemBase {

    // The driving motors of each module, arranged [fr, fl, rr, rl].
    private final TalonFX[] driveMotors = {
            new TalonFX(FR_DRIVE_MOTOR_PORT),
            new TalonFX(FL_DRIVE_MOTOR_PORT),
            new TalonFX(RR_DRIVE_MOTOR_PORT),
            new TalonFX(RL_DRIVE_MOTOR_PORT)};

    // The angle motors of each module, arranged [fr, fl, rr, rl].
    private final TalonSRX[] angleMotors = {
            new TalonSRX(FR_ANGLE_MOTOR_PORT),
            new TalonSRX(FL_ANGLE_MOTOR_PORT),
            new TalonSRX(RR_ANGLE_MOTOR_PORT),
            new TalonSRX(RL_ANGLE_MOTOR_PORT)};

    // The skews of each module relative to the robot's center, arranged [fr, fl, rr, rl].
    private final Translation2d[] wheelSkews = {
            new Translation2d(SKEW_X_FR_METERS, SKEW_Y_FR_METERS),
            new Translation2d(SKEW_X_FL_METERS, SKEW_Y_FL_METERS),
            new Translation2d(SKEW_X_RR_METERS, SKEW_Y_RR_METERS),
            new Translation2d(SKEW_X_RL_METERS, SKEW_Y_RL_METERS)
    };

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(wheelSkews);

    // The states of each module, speed [m/s] and angle (0deg - 360deg).
    private final SwerveModuleState moduleStates[] = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeedsRobotOriented = new ChassisSpeeds();

    private AHRS navx = new AHRS(NAVX_PORT);

    private double robotAngle; // Angle (0deg - 360deg) of the robot relative to itself.

    private final static SwerveSubsystem INSTANCE = new SwerveSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        return INSTANCE;
    }

    private SwerveSubsystem() {
        Arrays.stream(driveMotors).forEach(e -> {
            e.config_kP(0, DRIVE_KP);
            e.config_kI(0, DRIVE_KI);
            e.config_kD(0, DRIVE_KD);
        });

        Arrays.stream(angleMotors).forEach(e -> {
            e.config_kP(0, ANGLE_KP);
            e.config_kI(0, ANGLE_KI);
            e.config_kD(0, ANGLE_KD);
        });

        Arrays.stream(angleMotors).forEach(e -> {
            e.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 20);
            e.configFeedbackNotContinuous(NOT_CONTINUOUS_FEEDBACK, 20);
            e.setNeutralMode(NeutralMode.Brake);
        });

        for (int i = 0; i < 4; i++) {
            angleMotors[i].setInverted(INVERSIONS[i][0]);
            driveMotors[i].setInverted(INVERSIONS[i][1]);
            angleMotors[i].setSensorPhase(INVERSIONS[i][2]);
            driveMotors[i].setSensorPhase(INVERSIONS[i][3]);
        }
    }

    /**
     * Set the speed [m/s] of a module.
     *
     * @param driveMotor - TalonFX driving motor of the module.
     * @param speed      - Desired speed [m/s] of the module.
     */
    private void setSpeed(TalonFX driveMotor, double speed) {
        driveMotor.set(ControlMode.Velocity, speed * TICKS_PER_METER / 10);
    }

    /**
     * Get the speed [m/s] of a module.
     *
     * @param driveMotor - TalonFX driving motor of the module.
     * @return the speed [m/s] of the module.
     */
    private double getSpeed(TalonFX driveMotor) {
        return driveMotor.getSelectedSensorVelocity() / TICKS_PER_METER * 10;
    }

    /**
     * Set the angle (0deg - 360deg) of a module.
     *
     * @param angleMotor - TalonSRX angle motor of the module.
     * @param angle      - Desired angle (0deg - 360deg) of the module (angle is optimized in setModuleState).
     */
    private void setAngle(TalonSRX angleMotor, double angle) {
        PIDController errorCalculator = new PIDController(1, 0, 0);
        errorCalculator.enableContinuousInput(-180, 180);

        double error = errorCalculator.calculate(getAngle(angleMotor), angle);
        double percentOutput = UtilEverything.map(error, -90, 90, -1, 1);

        PIDController pidController = new PIDController(1, 0, 0);
        percentOutput = pidController.calculate(0, percentOutput);

        angleMotor.set(ControlMode.PercentOutput, -percentOutput); // I want cw to be positive, sue me.
    }

    /**
     * Get the angle of a module.
     *
     * @param angleMotor - TalonSRX angle motor of the module.
     * @return the angle (0deg - 360deg) of the module.
     */
    private double getAngle(TalonSRX angleMotor) {
        return angleMotor.getSelectedSensorPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Set the speed [m/s] and angle (0deg - 360deg) of each module to the desired state.
     *
     * @param desiredStates - Speed [m/s] and angle (0deg - 360deg) for each module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++) {
            desiredStates[i] = SwerveModuleState.optimize(desiredStates[i], new Rotation2d(Math.toRadians((getAngle(angleMotors[i])))));
            desiredStates[i].angle = new Rotation2d(Math.toRadians(UtilEverything.toPositive(desiredStates[i].angle.getDegrees())));
            setSpeed(driveMotors[i], desiredStates[i].speedMetersPerSecond);
            setAngle(angleMotors[i], desiredStates[i].angle.getDegrees());
        }
    }

    /**
     * Get the states, angle (0deg - 360deg) and speed [m/s], of the modules.
     *
     * @return the states, angle (0deg - 360deg) and speed [m/s], of the modules.
     */
    public SwerveModuleState[] getModuleStates() {
        return moduleStates;
    }

    /**
     * Set the chassis speeds of the robot relative to the field.
     *
     * @param desiredChassisSpeeds - Desired Chassis Speeds of the robot.
     */
    public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredChassisSpeeds.vxMetersPerSecond,
                desiredChassisSpeeds.vyMetersPerSecond,
                desiredChassisSpeeds.omegaRadiansPerSecond,
                new Rotation2d(Math.toRadians(robotAngle)));

        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        Arrays.stream(swerveModuleStates).forEach(e -> e.angle = new Rotation2d(Math.toRadians(UtilEverything.toPositive(e.angle.getDegrees()))));

        setModuleStates(swerveModuleStates);
    }

    /**
     * Set the chassis speeds of the robot relative to the robot.
     *
     * @param desiredChassisSpeeds - Desired Chassis Speeds of the robot.
     */
    public void setChassisSpeedsRobotOriented(ChassisSpeeds desiredChassisSpeeds) {
        SwerveModuleState[] desiredSwerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        Arrays.stream(desiredSwerveModuleStates).forEach(e -> e.angle = new Rotation2d(UtilEverything.toPositive(e.angle.getDegrees())));
        setModuleStates(desiredSwerveModuleStates);
    }

    /**
     * Get the chassis speeds of the robot relative to the field.
     *
     * @return the chassis speeds of the robot relative to the field.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Get the chassis speeds of the robot relative to the robot.
     *
     * @return the chassis speeds of the robot relative to the robot.
     */
    public ChassisSpeeds getChassisSpeedsRobotOriented() {
        return chassisSpeedsRobotOriented;
    }

    /**
     * Get the angle (0deg - 360deg) of the robot.
     *
     * @return the angle (0deg - 360deg) of the robot.
     */
    public double getRobotAngle() {
        return this.robotAngle;
    }

    /**
     * Update the states of each module.
     */
    private void updateStates() {
        for (int i = 0; i < 4; i++) {
            moduleStates[i].speedMetersPerSecond = getSpeed(driveMotors[i]);
            moduleStates[i].angle = new Rotation2d(Math.toRadians(getAngle(angleMotors[i])));
        }
    }


    /**
     * Update the chassis speeds of the robot relative to the field.
     */
    private void updateChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = swerveDriveKinematics.toChassisSpeeds(moduleStates);
        this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, new Rotation2d(robotAngle));
    }

    /**
     * Update the chassis speeds of the robot relative to the robot.
     */
    private void updateChassisSpeedsRobotOriented() {
        this.chassisSpeedsRobotOriented = swerveDriveKinematics.toChassisSpeeds(moduleStates);
    }

    /**
     * Update the angle (0deg - 360deg) of the robot.
     */
    private void updateRobotAngle() {
        robotAngle = UtilEverything.fromPOVtoUnitCircle(navx.getAngle() % 360);
    }

    /**
     * Set module angles (0deg - 360deg) to lock position.
     */
    public void lock() {
        SwerveModuleState[] lockStates = {
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(135))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(225))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(315)))
        };
        setModuleStates(lockStates);
    }

    @Override
    public void periodic() {
        updateStates();
        updateChassisSpeeds();
        updateChassisSpeedsRobotOriented();
        updateRobotAngle();
    }
}
