package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    private TalonFX m_turretController = new TalonFX(kTurretControllerId, kCANivoreCANbus);
    private TalonFX m_fourBarController = new TalonFX(kFourBarControllerId, kCANivoreCANbus);
    private CANcoder m_cancoder = new CANcoder(kCANcoderID, kCANivoreCANbus);

    private TurretPosition m_lastTurretPosition = TurretPosition.Forward;

    public enum FourBarHeight {
        Stowed,
        Ground,
        Low,
        Mid,
        High,
        NoChange
    }

    public enum TurretPosition {
        Forward,
        Left,
        Backward,
        Right,
        NoChange,
        Manual,
    }

    private DutyCycleOut m_turretOut = new DutyCycleOut(0);
    private PositionDutyCycle m_turretPosition = new PositionDutyCycle(0);
    private DutyCycleOut m_fourBarOut = new DutyCycleOut(0);

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.1;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.2;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        cfg.Slot0.kP = 0.1;
        
        m_turretController.getConfigurator().apply(cfg);
    }

    public CommandBase manualControlCommand(
            DoubleSupplier turretControl, DoubleSupplier fourBarControl, BooleanSupplier slowdownEnabled) {
        return new RunCommand(
                        () -> {
                            manualControl(turretControl.getAsDouble(), fourBarControl.getAsDouble(), slowdownEnabled.getAsBoolean());
                        },
                        this)
                .ignoringDisable(true);
    }

    public void manualControl(double turretControl, double fourBarControl, boolean slowdownEnabled) {
        double speed = 1;
        if (slowdownEnabled) {
            speed = 0.5;
        }
        m_turretController.setControl(m_turretOut.withOutput(turretControl * 0.1 * speed));
        m_fourBarController.setControl(m_fourBarOut.withOutput(fourBarControl * 0.2 * speed));
    }

    public CommandBase setTurretCommand(
            Supplier<TurretPosition> turretPosition) {
        return new RunCommand(
                        () -> {
                            setTurretPosition(turretPosition.get());
                        },
                        this)
                .ignoringDisable(true);
    }

    public void setTurretPosition(TurretPosition position)
    {
        if(position != TurretPosition.NoChange)
        {
            m_lastTurretPosition = position;
        }

        switch(m_lastTurretPosition)
        {
            case Manual:
            case NoChange:
                /* Shouldn't happen */
                break;
            case Forward:
                m_turretController.setControl(m_turretPosition.withPosition(0.785));
                break;
            case Left:
                m_turretController.setControl(m_turretPosition.withPosition(0.54736));
                break;
            case Backward:
                m_turretController.setControl(m_turretPosition.withPosition(0.2714));
                break;
            case Right:
                m_turretController.setControl(m_turretPosition.withPosition(1.04));
                break;
        }
    }
}
