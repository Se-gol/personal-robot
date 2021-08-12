package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveExperimentalTankDriveDefaultCommand extends CommandBase {

    private double rightX;
    private double rightY;
    private double leftX;
    private double leftY;
    private final XboxController xboxController = RobotContainer.xboxController;
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public SwerveExperimentalTankDriveDefaultCommand() {
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        rightX = xboxController.getX(GenericHID.Hand.kRight);
        rightY = xboxController.getY(GenericHID.Hand.kRight);
        leftX = xboxController.getX(GenericHID.Hand.kLeft);
        leftY = xboxController.getY(GenericHID.Hand.kLeft);

        SwerveModuleState[] desiredSwerveModuleStates = {
                new SwerveModuleState(rightY, new Rotation2d(Math.toRadians(90))),
                new SwerveModuleState(leftY, new Rotation2d(Math.toRadians(90))),
                new SwerveModuleState(leftY, new Rotation2d(Math.toRadians(90))),
                new SwerveModuleState(leftY, new Rotation2d(Math.toRadians(90)))
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
