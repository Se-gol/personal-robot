package frc.robot.commands.practical;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.FunnelConstants.*;
import static frc.robot.Constants.ConveyorConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class OuttakePowerCellsCommand extends CommandBase {
    private final FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
    private final ConveyorSubsystem conveyorSubsystem = ConveyorSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public OuttakePowerCellsCommand() {
        addRequirements(funnelSubsystem, conveyorSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        funnelSubsystem.setPower(-FUNNEL_POWER);
        conveyorSubsystem.setPower(-CONVEYOR_POWER);
        shooterSubsystem.setPower(-OUTTAKE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        funnelSubsystem.setPower(0);
        conveyorSubsystem.setPower(0);
        shooterSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
