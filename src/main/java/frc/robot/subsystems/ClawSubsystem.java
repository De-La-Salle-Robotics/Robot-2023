package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ClawConstants.*;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class ClawSubsystem extends SubsystemBase {
    private final double kSuckPower = 0.6;
    private final double kBlowPower = -0.6;
    private final double kNeutralPower = 0.0;

    private final TalonFX m_clawMotor = new TalonFX(kClawTalonId, kCANivoreCANbus);

    private final DutyCycleOut m_clawPower = new DutyCycleOut(kNeutralPower);

    public enum ClawState {
        Sucking,
        Blowing,
        Neutral,
        NoChange
    }

    public ClawSubsystem() {}

    public CommandBase controlClaw(Supplier<ClawState> clawState) {
        return new RunCommand(
                        () -> {
                            switch (clawState.get()) {
                                case Sucking:
                                    suck();
                                    break;
                                case Blowing:
                                    blow();
                                    break;
                                case Neutral:
                                    neutral();
                                    break;
                                case NoChange:
                                    break;
                            }
                        },
                        this)
                .ignoringDisable(true);
    }

    public void suck() {
        m_clawMotor.setControl(m_clawPower.withOutput(kSuckPower));
    }

    public void blow() {
        m_clawMotor.setControl(m_clawPower.withOutput(kBlowPower));
    }

    public void neutral() {
        m_clawMotor.setControl(m_clawPower.withOutput(kNeutralPower));
    }
}
