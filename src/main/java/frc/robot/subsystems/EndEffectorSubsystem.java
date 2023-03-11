package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class EndEffectorSubsystem extends SubsystemBase {
    private final double kServoDown = -1;
    private final double kServoUp = 0.85;
    private final double kVaccuumRunningPower = 0.6;
    private final double kVaccuumStopPower = 0;

    public enum VaccuumState {
        Sucking,
        Released,
        NoChange
    }

    public enum EffectorState {
        Up,
        Down,
        NoChange
    }

    private PWM m_servo = new PWM(kServoPort);

    private Solenoid m_releaseSolenoid =
            new Solenoid(PneumaticsModuleType.CTREPCM, kReleaseSolenoidPort);

    private WPI_VictorSPX m_vaccuumPump = new WPI_VictorSPX(kVaccuumPumpId);

    public EndEffectorSubsystem() {}

    public CommandBase controlEffector(
            Supplier<VaccuumState> vaccuumState, Supplier<EffectorState> effectorState) {
        return new RunCommand(
                        () -> {
                            switch (vaccuumState.get()) {
                                case Sucking:
                                    suck();
                                    break;
                                case Released:
                                    release();
                                    break;
                                case NoChange:
                                    break;
                            }
                            switch (effectorState.get()) {
                                case Up:
                                    putServoUp();
                                    break;
                                case Down:
                                    putServoDown();
                                    break;
                                case NoChange:
                                    break;
                            }
                        },
                        this)
                .ignoringDisable(true);
    }

    public void putServoDown() {
        m_servo.setPosition(kServoDown);
    }

    public void putServoUp() {
        m_servo.setPosition(kServoUp);
    }

    public void suck() {
        m_releaseSolenoid.set(false);

        runVaccuum();
    }

    public void release() {
        m_releaseSolenoid.set(true);

        stopVaccuum();
    }

    public void runVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumRunningPower);
    }

    public void stopVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumStopPower);
    }
}
