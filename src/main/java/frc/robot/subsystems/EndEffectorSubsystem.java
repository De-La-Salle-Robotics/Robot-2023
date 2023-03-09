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
    private final double kConeServoDown = -1;
    private final double kConeServoUp = 1;
    private final double kCubeServoDown = -1;
    private final double kCubeServoUp = 1;
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

    private PWM m_coneServo = new PWM(kConeServorPort);
    private PWM m_cubeServo = new PWM(kCubeServorPort);
    private Solenoid m_coneSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, kConeSolenoidPort);
    private Solenoid m_cubeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, kCubeSolenoidPort);

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
        m_cubeServo.setPosition(kCubeServoDown);
    }

    public void putCubeServoUp() {
        m_cubeServo.setPosition(kCubeServoUp);
    }

    public void suctionCone() {
        /* If we're setting cone solenoid, we need to make sure cube is off */
        m_cubeSolenoid.set(false);

        m_coneSolenoid.set(true);
        runVaccuum();
    }

    public void releaseCone() {
        m_coneSolenoid.set(false);
        stopVaccuum();
    }

    public void suctionCube() {
        /* If we're setting cube solenoid, we need to make sure cone is off */
        m_coneSolenoid.set(false);

        m_cubeSolenoid.set(true);
        runVaccuum();
    }

    public void releaseCube() {
        m_cubeSolenoid.set(false);
        stopVaccuum();
    }

    public void runVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumRunningPower);
    }

    public void stopVaccuum() {
        m_vaccuumPump.set(ControlMode.PercentOutput, kVaccuumStopPower);
    }
}
