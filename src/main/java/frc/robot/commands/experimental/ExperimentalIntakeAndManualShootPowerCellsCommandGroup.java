package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.practical.IntakePowerCellsCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ExperimentalIntakeAndManualShootPowerCellsCommandGroup extends ParallelCommandGroup {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public ExperimentalIntakeAndManualShootPowerCellsCommandGroup(double velocity, double angle) {
        addCommands(
                new ExperimentalManualShootPowerCellsCommand(velocity, angle),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooterSubsystem.haveFlywheelAndHoodReachedSetPoint(velocity, shooterSubsystem.angleToTicks(angle))),
                        new IntakePowerCellsCommandGroup())
        );
    }
}
