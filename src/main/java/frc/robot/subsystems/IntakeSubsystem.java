package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.IntakePorts.*;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(MOTOR_PORT);
    private final Solenoid piston = new Solenoid(PISTON_PORT);

    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    private IntakeSubsystem() {
        motor.setInverted(MOTOR_INVERTED);
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public enum PistonMode {
        ENGAGED(true), DISENGAGED(false);

        private boolean value;

        PistonMode(boolean value) {
            this.value = value;
        }
    }

    public void togglePiston() {
        piston.toggle();
    }

    public PistonMode getPistonMode() {
        if (piston.get() == PistonMode.ENGAGED.value) return PistonMode.ENGAGED;
        return PistonMode.DISENGAGED;
    }

    public void setPistonMode(PistonMode pistonMode) {
        piston.set(pistonMode.value);
    }


}

