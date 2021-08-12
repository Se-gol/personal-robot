package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.ConveyorPorts.*;

public class ConveyorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(MOTOR_PORT);

    private final static ConveyorSubsystem INSTANCE = new ConveyorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ConveyorSubsystem getInstance() {
        return INSTANCE;
    }

    private ConveyorSubsystem() {
        motor.setInverted(MOTOR_INVERTED);
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
}

