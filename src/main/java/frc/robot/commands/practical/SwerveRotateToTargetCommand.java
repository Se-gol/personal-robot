package frc.robot.commands.practical;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.UtilEverything;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.SwerveConstants.ROTATION_SPEED_MULTIPLIER;

public class SwerveRotateToTargetCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final PhotonVisionSubsystem photonVisionSubsystem = PhotonVisionSubsystem.getInstance();

    public SwerveRotateToTargetCommand() {
        addRequirements(swerveSubsystem, photonVisionSubsystem);
    }

    @Override
    public void execute() {
        double error = photonVisionSubsystem.getTargetXError().orElse(0);
        double mappedError = UtilEverything.map(error, -180, 180, -1, 1);
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, mappedError * ROTATION_SPEED_MULTIPLIER));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (photonVisionSubsystem.getTargetXError().isEmpty()) return false;
        return Math.abs(UtilEverything.toPositive(photonVisionSubsystem.getTargetXError().getAsDouble()) - swerveSubsystem.getRobotAngle()) < 1;
    }
}
