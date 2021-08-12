package frc.robot.commands.practical;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeAndShootPowerCellsCommandGroup extends ParallelCommandGroup {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final PhotonVisionSubsystem photonVisionSubsystem = PhotonVisionSubsystem.getInstance();


    public IntakeAndShootPowerCellsCommandGroup() {
        addCommands(
                new ShootPowerCellsCommand(),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooterSubsystem.haveFlywheelAndHoodReachedSetPoint(
                                shooterSubsystem.calculateVelocity(photonVisionSubsystem.getDistanceFromTarget().orElse(0)),
                                shooterSubsystem.calculateAngle(photonVisionSubsystem.getDistanceFromTarget().orElse(0)))),

                        new IntakePowerCellsCommandGroup())
        );
    }
}
