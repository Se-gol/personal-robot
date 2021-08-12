package frc.robot.commands.practical;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.IntakeConstants.*;

public class ActivateIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ActivateIntakeCommand() {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPistonMode(IntakeSubsystem.PistonMode.ENGAGED);
    }

    @Override
    public void execute() {
        intakeSubsystem.setPower(INTAKE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setPower(0);
        intakeSubsystem.setPistonMode(IntakeSubsystem.PistonMode.DISENGAGED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
