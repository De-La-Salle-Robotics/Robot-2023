package frc.robot.subsystems;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CtrUtils;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ClawSubsystem extends SubsystemBase {
    private final double kSuckPower = 0.6;
    private final double kBlowPower = -0.6;
    private final double kNeutralPower = 0.0;

    private final WPI_TalonFX m_clawMotor = new WPI_TalonFX(kClawTalonId, "rio");

    public enum ClawState {
        Sucking,
        Blowing,
        Neutral,
        NoChange
    }

    public ClawSubsystem() {
        CtrUtils.runUntilSuccessWithTimeoutV5((timeout)->{return m_clawMotor.configFactoryDefault(timeout);}, 100, 5);
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
        m_clawMotor.set(ControlMode.PercentOutput, kSuckPower);
    }

    public void blow() {
        m_clawMotor.set(ControlMode.PercentOutput, kBlowPower);
    }

    public void neutral() {
        m_clawMotor.set(ControlMode.PercentOutput, kNeutralPower);
    }
}
