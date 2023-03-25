package frc.robot.subsystems;

import static frc.robot.Constants.ClawConstants.*;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CtrUtils;
import java.util.function.Supplier;

public class ClawSubsystem extends SubsystemBase {
    private final double kMaxDutyCycle = 0.5;
    private final double kSuckPower = 200;
    private final double kBlowPower = -200;
    private final double kNeutralPower = 0.0;

    private final TalonFX m_clawMotor = new TalonFX(kClawTalonId, "rio");
    private final TorqueCurrentFOC m_torqueDemand = new TorqueCurrentFOC(0);

    public enum ClawState {
        Sucking,
        Blowing,
        Neutral,
        NoChange
    }

    public ClawSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        CtrUtils.runUntilSuccessWithTimeoutPro(
                (timeout) -> {
                    return m_clawMotor.getConfigurator().apply(cfg, timeout);
                },
                100,
                5);
    }

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
        m_clawMotor.setControl(
                m_torqueDemand.withOutput(kSuckPower).withMaxAbsDutyCycle(kMaxDutyCycle));
    }

    public void blow() {
        m_clawMotor.setControl(
                m_torqueDemand.withOutput(kBlowPower).withMaxAbsDutyCycle(kMaxDutyCycle));
    }

    public void neutral() {
        m_clawMotor.setControl(
                m_torqueDemand.withOutput(kNeutralPower).withMaxAbsDutyCycle(kMaxDutyCycle));
    }
}
