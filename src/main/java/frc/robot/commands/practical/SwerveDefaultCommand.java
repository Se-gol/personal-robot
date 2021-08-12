package frc.robot.commands.practical;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveDefaultCommand extends CommandBase {

    private double rightX;
    private double rightY;
    private double leftX;
    private double leftY;
    private final XboxController xboxController = RobotContainer.xboxController;
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public SwerveDefaultCommand() {
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        rightX = xboxController.getX(GenericHID.Hand.kRight);
        rightY = xboxController.getY(GenericHID.Hand.kRight);
        leftX = xboxController.getX(GenericHID.Hand.kLeft);
        leftY = xboxController.getY(GenericHID.Hand.kLeft);

        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(
                leftX * DRIVE_SPEED_MULTIPLIER,
                leftY * DRIVE_SPEED_MULTIPLIER,
                rightX * ROTATION_SPEED_MULTIPLIER);

        swerveSubsystem.setChassisSpeeds(desiredChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.lock();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
