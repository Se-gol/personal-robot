package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.UtilEverything;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.SwerveConstants.DRIVE_SPEED_MULTIPLIER;
import static frc.robot.Constants.SwerveConstants.ROTATION_SPEED_MULTIPLIER;

public class SwerveExperimentalDriveAndRotateToJoystickAngle extends CommandBase {

    private double rightX;
    private double rightY;
    private double leftX;
    private double leftY;
    private final XboxController xboxController = RobotContainer.xboxController;
    private final PIDController errorCalculator = new PIDController(1, 0, 0);
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public SwerveExperimentalDriveAndRotateToJoystickAngle() {
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        rightX = xboxController.getX(GenericHID.Hand.kRight);
        rightY = xboxController.getY(GenericHID.Hand.kRight);
        leftX = xboxController.getX(GenericHID.Hand.kLeft);
        leftY = xboxController.getY(GenericHID.Hand.kLeft);

        double angle = Math.toDegrees(Math.atan2(rightY, rightX));
        errorCalculator.enableContinuousInput(-180, 180);
        double error = errorCalculator.calculate(swerveSubsystem.getRobotAngle(), angle);
        double mappedError = UtilEverything.map(error, -180, 180, -1, 1);


        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(
                leftX * DRIVE_SPEED_MULTIPLIER,
                leftY * DRIVE_SPEED_MULTIPLIER,
                -mappedError * ROTATION_SPEED_MULTIPLIER);

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
