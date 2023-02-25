package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class EndEffectorSubsystem extends SubsystemBase {
    private final double kConeServoDown = 0;
    private final double kConeServoUp = 1;
    private final double kCubeServoDown = 0;
    private final double kCubeServoUp = 1;
    private final double kVaccuumRunningPower = 0.3;
    private final double kVaccuumStopPower = 0;

    public enum VaccuumState {
        Sucking,
        Released,
        Stopped,
        NoChange
    }

    public enum EffectorState {
        Up,
        Down,
        NoChange
    }

    private PWM m_coneServo = new PWM(kConeServorPort);
    private PWM m_cubeSero = new PWM(kCubeServorPort);
    private DoubleSolenoid m_coneSolenoid =
            new DoubleSolenoid(
                    PneumaticsModuleType.CTREPCM, kConeSolenoidPortForward, kConeSolenoidPortReverse);
    private DoubleSolenoid m_cubeSolenoid =
            new DoubleSolenoid(
                    PneumaticsModuleType.CTREPCM, kCubeSolenoidPortForward, kCubeSolenoidPortReverse);

    private WPI_VictorSPX m_vaccuumPump = new WPI_VictorSPX(kVaccuumPumpId);

    public EndEffectorSubsystem() {}

    public CommandBase controlCubeSide(
            Supplier<VaccuumState> vaccuumState, Supplier<EffectorState> effectorState) {
        return new RunCommand(
                        () -> {
                            switch (vaccuumState.get()) {
                                case Sucking:
                                    suctionCube();
                                    break;
                                case Stopped:
                                    holdCube();
                                    break;
                                case Released:
                                    releaseCube();
                                    break;
                                case NoChange:
                                    break;
                            }
                            switch (effectorState.get()) {
                                case Up:
                                    putCubeServoUp();
                                    break;
                                case Down:
                                    putCubeServoDown();
                                    break;
                                case NoChange:
                                    break;
                            }
                        },
                        this)
                .ignoringDisable(true);
    }

    public CommandBase controlConeSide(
            Supplier<VaccuumState> vaccuumState, Supplier<EffectorState> effectorState) {
        return new RunCommand(
                        () -> {
                            switch (vaccuumState.get()) {
                                case Sucking:
                                    suctionCone();
                                    break;
                                case Stopped:
                                    holdCone();
                                    break;
                                case Released:
                                    releaseCone();
                                    break;
                                case NoChange:
                                    break;
                            }
                            switch (effectorState.get()) {
                                case Up:
                                    putConeServoUp();
                                    break;
                                case Down:
                                    putConeServoDown();
                                    break;
                                case NoChange:
                                    break;
                            }
                        },
                        this)
                .ignoringDisable(true);
    }

    public void putConeServoDown() {
        m_coneServo.setPosition(kConeServoDown);
    }

    public void putConeServoUp() {
        m_coneServo.setPosition(kConeServoUp);
    }

    public void putCubeServoDown() {
        m_cubeSero.setPosition(kCubeServoDown);
    }

    public void putCubeServoUp() {
        m_cubeSero.setPosition(kCubeServoUp);
    }

    public void suctionCone() {
        /* If we're setting cone solenoid, we need to make sure cube is off */
        m_cubeSolenoid.set(Value.kOff);

        m_coneSolenoid.set(Value.kForward);
    }

    public void holdCone() {
        m_coneSolenoid.set(Value.kOff);
    }

    public void releaseCone() {
        /* If we're setting cone solenoid, we need to make sure cube is off */
        m_cubeSolenoid.set(Value.kOff);

        m_coneSolenoid.set(Value.kReverse);
    }

    public void suctionCube() {
        /* If we're setting cube solenoid, we need to make sure cone is off */
        m_coneSolenoid.set(Value.kOff);

        m_cubeSolenoid.set(Value.kForward);
    }

    public void holdCube() {
        m_cubeSolenoid.set(Value.kOff);
    }

    public void releaseCube() {
        /* If we're setting cube solenoid, we need to make sure cone is off */
        m_coneSolenoid.set(Value.kOff);

        m_cubeSolenoid.set(Value.kReverse);
    }

    public void runVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumRunningPower);
    }

    public void stopVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumStopPower);
    }
}
