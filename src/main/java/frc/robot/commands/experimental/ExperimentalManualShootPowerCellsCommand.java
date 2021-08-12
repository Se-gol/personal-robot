package frc.robot.commands.experimental;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ExperimentalManualShootPowerCellsCommand extends CommandBase {
    private final double velocity;
    private final double angle;
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final XboxController xboxController = RobotContainer.xboxController;

    public ExperimentalManualShootPowerCellsCommand(double velocity, double angle) {
        this.velocity = velocity;
        this.angle = angle;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setHoodPosition(shooterSubsystem.angleToTicks(angle));
        shooterSubsystem.setVelocity(velocity);

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
