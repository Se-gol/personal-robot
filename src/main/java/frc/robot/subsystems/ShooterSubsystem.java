package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Ports.ShooterPorts.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(SHOOTER_MOTOR_PORT);
    private final TalonFX shooterSlave = new TalonFX(SHOOTER_SLAVE_PORT);
    private final TalonSRX hoodMotor = new TalonSRX(HOOD_MOTOR_PORT);

    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    private ShooterSubsystem() {
        shooterSlave.follow(shooterMotor);

        shooterMotor.config_kP(0, SHOOTER_KP);
        shooterMotor.config_kP(0, SHOOTER_KI);
        shooterMotor.config_kP(0, SHOOTER_KD);

        hoodMotor.config_kP(0, HOOD_KP);
        hoodMotor.config_kP(0, HOOD_KI);
        hoodMotor.config_kP(0, HOOD_KD);

        shooterMotor.setInverted(SHOOTER_MOTOR_INVERTED);
        shooterMotor.setSensorPhase(SHOOTER_SENSOR_INVERTED);
        shooterSlave.setInverted(SHOOTER_SLAVE_INVERTED);

        hoodMotor.setInverted(HOOD_INVERTED);
        hoodMotor.setSensorPhase(HOOD_SENSOR_INVERTED);
        hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
    }

    public double getVelocity() {
        return shooterMotor.getSelectedSensorVelocity() * 10 / TICKS_PER_REVOLUTION;
    }

    public void setVelocity(double velocity) {
        shooterMotor.set(ControlMode.Velocity, velocity / 10 * TICKS_PER_REVOLUTION);
    }

    public void setPower(double power) {
        shooterMotor.set(ControlMode.PercentOutput, power);
    }

    public void setHoodPosition(int position) {
        hoodMotor.set(ControlMode.MotionMagic, position);
    }

    private double calculateAngleDeg(double distance) {
        double TARGET_HEIGHT = 2.5;
        double SHOOTER_HEIGHT = 0.66;
        return Math.toDegrees(Math.atan(2 * (TARGET_HEIGHT - SHOOTER_HEIGHT) / distance));
    }

    public int angleToTicks(double angle) {
        return (int) ((angle - 90) * HOOD_MAX_POSITION / -90.0);
    }

    private double calculateVelocityMS(double distance) {
        return Math.sqrt(G * Math.pow(distance, 2) / (2 * Math.pow(Math.cos(Math.toRadians(calculateAngleDeg(distance))), 2)
                * (distance * Math.tan(Math.toRadians(calculateAngleDeg(distance))) - (TARGET_HEIGHT - SHOOTER_HEIGHT))));
    }

    private double velocityToRPS(double velocity) {
        return velocity / (2 * Math.PI * FLYWHEEL_RADIUS);
    }

    public int calculateAngle(double distance) {
        return angleToTicks(calculateAngleDeg(distance));
    }

    public double calculateVelocity(double distance) {
        return velocityToRPS(calculateVelocityMS(distance)) * FLYWHEEL_SPEED_MULTIPLIER;
    }

    private boolean hasFlywheelReachedSetPoint(double setPoint) {
        return Math.abs(setPoint - getVelocity()) <= VELOCITY_TOLERANCE;
    }

    private boolean hasHoodReachedSetPoint(double setPoint) {
        return Math.abs(setPoint - hoodMotor.getSelectedSensorPosition()) <= HOOD_TICKS_TOLERANCE;
    }

    public boolean haveFlywheelAndHoodReachedSetPoint(double flywheelSetPoint, double hoodSetPoint) {
        return hasFlywheelReachedSetPoint(flywheelSetPoint) && hasHoodReachedSetPoint(hoodSetPoint);
    }
}

