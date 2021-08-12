package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveExperimentalOmniDriveDefaultCommand extends CommandBase {

    private double rightX;
    private double rightY;
    private double leftX;
    private double leftY;
    private final XboxController xboxController = RobotContainer.xboxController;
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public SwerveExperimentalOmniDriveDefaultCommand() {
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        rightX = xboxController.getX(GenericHID.Hand.kRight);
        rightY = xboxController.getY(GenericHID.Hand.kRight);
        leftX = xboxController.getX(GenericHID.Hand.kLeft);
        leftY = xboxController.getY(GenericHID.Hand.kLeft);

        double frSpeed = -leftY + leftX + rightX / 2;
        double flSpeed = leftY + leftX + rightX / 2;
        double rrSpeed = leftY + leftX - rightX / 2;
        double rlSpeed = -leftY + leftX - rightX / 2;

        SwerveModuleState[] desiredSwerveModuleStates = {
                new SwerveModuleState(frSpeed, new Rotation2d(Math.toRadians(135))),
                new SwerveModuleState(flSpeed, new Rotation2d(Math.toRadians(225))),
                new SwerveModuleState(rrSpeed, new Rotation2d(Math.toRadians(45))),
                new SwerveModuleState(rlSpeed, new Rotation2d(Math.toRadians(315)))
        };
        swerveSubsystem.setModuleStates(desiredSwerveModuleStates);
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
