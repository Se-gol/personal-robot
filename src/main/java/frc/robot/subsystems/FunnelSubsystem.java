package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.FunnelPorts.*;

public class FunnelSubsystem extends SubsystemBase {
    private VictorSPX motor = new VictorSPX(MOTOR_PORT);
    private Solenoid piston = new Solenoid(PISTON_PORT);

    private final static FunnelSubsystem INSTANCE = new FunnelSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static FunnelSubsystem getInstance() {
        return INSTANCE;
    }

    private FunnelSubsystem() {
        motor.setInverted(MOTOR_INVERTED);
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void togglePiston() {
        piston.toggle();
    }
}

