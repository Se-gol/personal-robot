package frc.robot.commands.practical;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakePowerCellsCommandGroup extends ParallelCommandGroup {
    public IntakePowerCellsCommandGroup() {
        addCommands(
//                new ActivateIntakeCommand(),
                new ActivateFunnelCommand(),
                new ActivateConveyorCommand()
        );
    }
}
