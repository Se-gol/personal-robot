package frc.robot.commands.practical;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPowerCellsCommand extends CommandBase {
    XboxController xboxController = RobotContainer.xboxController;
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final PhotonVisionSubsystem photonVisionSubsystem = PhotonVisionSubsystem.getInstance();

    public ShootPowerCellsCommand() {
        addRequirements(shooterSubsystem, photonVisionSubsystem);
    }

    @Override
    public void execute() {
        if (photonVisionSubsystem.getDistanceFromTarget().isEmpty()) return;

        shooterSubsystem.setHoodPosition(shooterSubsystem.calculateAngle(photonVisionSubsystem.getDistanceFromTarget().getAsDouble()));
        shooterSubsystem.setVelocity(shooterSubsystem.calculateVelocity(photonVisionSubsystem.getDistanceFromTarget().getAsDouble()));

        xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
        xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
