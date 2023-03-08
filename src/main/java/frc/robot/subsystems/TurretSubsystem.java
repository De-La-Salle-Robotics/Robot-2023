package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
    private TalonFX m_turretController = new TalonFX(kTurretControllerId, kCANivoreCANbus);
    private TalonFX m_fourBarController = new TalonFX(kFourBarControllerId, kCANivoreCANbus);

    enum FourBarHeight {
        Stowed,
        Ground,
        Low,
        Mid,
        High,
        NoChange
    }

    enum TurretPosition {
        Forward,
        Left,
        Backward,
        NoChange
    }

    private DutyCycleOut m_turretOut = new DutyCycleOut(0);
    private DutyCycleOut m_fourBarOut = new DutyCycleOut(0);

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
}
