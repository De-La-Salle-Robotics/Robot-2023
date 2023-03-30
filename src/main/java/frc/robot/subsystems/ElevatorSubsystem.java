package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CtrUtils;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
    private final double kElevatorLow = -43;
    private final double kElevatorMid = 7.84;
    private final double kElevatorHigh = 17.4;
    private final double kExtenderIn = 0;
    private final double kExtenderMid = 19.4;
    private final double kExtenderOut = 36.7;

    private final TalonFX m_elevatorMotor = new TalonFX(kElevatorTalonId, kCANivoreCANbus);
    private final TalonFX m_extenderMotor = new TalonFX(kExtenderTalonId, kCANivoreCANbus);

    private final DutyCycleOut m_elevatorPower = new DutyCycleOut(0);
    private final DutyCycleOut m_extenderPower = new DutyCycleOut(0);

    private final MotionMagicVoltage m_elevatorTarget = new MotionMagicVoltage(0);
    private final MotionMagicVoltage m_extenderTarget = new MotionMagicVoltage(0);

    private boolean m_closedLoop = false;

    public enum DesiredLocation {
        Stow,
        LowScore,
        MiddleScore,
        HighScore,
        LoadStationPickup,
        GroundPickup,
        NoChange
    }

    private final Map<DesiredLocation, Pair<Double, Double>> m_stateLocations =
            new HashMap<DesiredLocation, Pair<Double, Double>>();

    public ElevatorSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotionMagic.MotionMagicAcceleration = 100;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 200;

        cfg.Slot0.kP = 20;

        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentThreshold = 40;
        cfg.CurrentLimits.SupplyTimeThreshold = 0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        CtrUtils.runUntilSuccessWithTimeoutPro(
                (timeout) -> {
                    return m_elevatorMotor.getConfigurator().apply(cfg);
                },
                0.1,
                5);
        CtrUtils.runUntilSuccessWithTimeoutPro(
                (timeout) -> {
                    return m_extenderMotor.getConfigurator().apply(cfg);
                },
                0.1,
                5);

        m_stateLocations.put(DesiredLocation.Stow, new Pair<Double, Double>(kElevatorLow, kExtenderIn));
        m_stateLocations.put(
                DesiredLocation.LowScore, new Pair<Double, Double>(kElevatorLow, kExtenderIn));
        m_stateLocations.put(
                DesiredLocation.MiddleScore, new Pair<Double, Double>(kElevatorMid, kExtenderMid));
        m_stateLocations.put(
                DesiredLocation.HighScore, new Pair<Double, Double>(kElevatorHigh, kExtenderOut));
        m_stateLocations.put(
                DesiredLocation.LoadStationPickup, new Pair<Double, Double>(kElevatorMid, kExtenderOut));
        m_stateLocations.put(
                DesiredLocation.GroundPickup, new Pair<Double, Double>(kElevatorLow, kExtenderMid));

        reZeroEverything();
    }

    public void reZeroEverything() {
        CtrUtils.runUntilSuccessWithTimeoutPro(
                (timeout) -> {
                    return m_elevatorMotor.setRotorPosition(0);
                },
                0.1,
                5);
        CtrUtils.runUntilSuccessWithTimeoutPro(
                (timeout) -> {
                    return m_extenderMotor.setRotorPosition(0);
                },
                0.1,
                5);
    }

    public CommandBase controlElevatorCommand(
            DoubleSupplier elevatorPower,
            DoubleSupplier extenderPower,
            Supplier<DesiredLocation> target) {
        return new RunCommand(
                        () -> {
                            /* Two stages, first determine if we should close loop, then control */
                            double elevPower = elevatorPower.getAsDouble();
                            double extePower = extenderPower.getAsDouble();
                            var targetState = target.get();

                            if (Math.abs(elevPower) > 0.1 || Math.abs(extePower) > 0.1) {
                                /* We're manually overriding */
                                m_closedLoop = false;
                            } else if (targetState != DesiredLocation.NoChange) {
                                /* We're targetting a position */
                                m_closedLoop = true;
                            }

                            if (m_closedLoop) {
                                /* If we're closed looping, then set the closed loop target */
                                setClosedLoop(targetState);
                            } else {
                                /* Otherwise use the manual control */
                                manualControlElevator(elevPower, extePower);
                            }
                        },
                        this)
                .ignoringDisable(true);
    }

    public void manualControlElevator(double elevatorPower, double extenderPower) {
        m_elevatorMotor.setControl(m_elevatorPower.withOutput(elevatorPower));
        m_extenderMotor.setControl(m_extenderPower.withOutput(extenderPower));
    }

    public void setClosedLoop(DesiredLocation targetLocation) {
        /* Don't change anything if we're nochange */
        /* This also handles if we move out of "manual" control */
        if (targetLocation == DesiredLocation.NoChange) return;

        var targets = m_stateLocations.get(targetLocation);
        m_elevatorMotor.setControl(m_elevatorTarget.withPosition(targets.getFirst()));
        m_extenderMotor.setControl(m_extenderTarget.withPosition(targets.getSecond()));
    }
}
