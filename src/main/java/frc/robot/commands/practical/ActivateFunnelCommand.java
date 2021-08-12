package frc.robot.commands.practical;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FunnelSubsystem;

import static frc.robot.Constants.FunnelConstants.*;

public class ActivateFunnelCommand extends CommandBase {

    private final FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
    private final Timer timer = new Timer();

    public ActivateFunnelCommand() {
        addRequirements(funnelSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        funnelSubsystem.setPower(FUNNEL_POWER);
        if (timer.hasPeriodPassed(PISTON_PERIOD)) {
            funnelSubsystem.togglePiston();
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        funnelSubsystem.setPower(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
