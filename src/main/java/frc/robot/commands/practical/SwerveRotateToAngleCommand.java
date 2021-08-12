package frc.robot.commands.practical;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.UtilEverything;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveRotateToAngleCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final double targetAngle;

    private final PIDController errorCalculator = new PIDController(1, 0, 0);

    public SwerveRotateToAngleCommand(double targetAngle) {
        this.targetAngle = targetAngle;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        errorCalculator.enableContinuousInput(-180, 180);
        double error = errorCalculator.calculate(swerveSubsystem.getRobotAngle(), targetAngle);
        double mappedError = UtilEverything.map(error, -180, 180, -1, 1);
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, mappedError * ROTATION_SPEED_MULTIPLIER));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetAngle - swerveSubsystem.getRobotAngle()) < 1;
    }
}
