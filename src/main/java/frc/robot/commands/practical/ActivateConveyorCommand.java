package frc.robot.commands.practical;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

import static frc.robot.Constants.ConveyorConstants.*;

public class ActivateConveyorCommand extends CommandBase {

    private ConveyorSubsystem conveyorSubsystem = ConveyorSubsystem.getInstance();

    public ActivateConveyorCommand() {
        addRequirements(conveyorSubsystem);
    }

    @Override
    public void execute() {
        conveyorSubsystem.setPower(CONVEYOR_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        conveyorSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
