// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.experimental.SwerveExperimentalCombinedAutoRotateAndRegularRotationDefaultCommand;
import frc.robot.commands.practical.IntakeAndShootPowerCellsCommandGroup;
import frc.robot.commands.practical.ShootPowerCellsCommand;
import frc.robot.commands.practical.SwerveRotateToAngleCommand;
import frc.robot.subsystems.*;

import static frc.robot.Ports.XBOX_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final PhotonVisionSubsystem photonVisionSubsystem = PhotonVisionSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
    private final ConveyorSubsystem conveyorSubsystem = ConveyorSubsystem.getInstance();

    public static XboxController xboxController = new XboxController(XBOX_CONTROLLER_PORT);
    private final Trigger povCheck = new Trigger(() -> xboxController.getPOV() != -1);
    private final Trigger RT = new Trigger(() -> xboxController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.3);


    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommand();
    }

    private void configureButtonBindings() {
        povCheck.whileActiveOnce(new SwerveRotateToAngleCommand(UtilEverything.fromPOVtoUnitCircle(xboxController.getPOV())));
        RT.whileActiveOnce(new IntakeAndShootPowerCellsCommandGroup());
    }

    public void configureDefaultCommand() {
        swerveSubsystem.setDefaultCommand(new SwerveExperimentalCombinedAutoRotateAndRegularRotationDefaultCommand());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
