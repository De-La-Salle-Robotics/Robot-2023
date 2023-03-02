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
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
    private TalonFX m_turretController = new TalonFX(kTurretControllerId, kCANivoreCANbus);
    private TalonFX m_fourBarController = new TalonFX(kFourBarControllerId, kCANivoreCANbus);

    private DutyCycleOut m_turretOut = new DutyCycleOut(0);
    private DutyCycleOut m_fourBarOut = new DutyCycleOut(0);

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_turretController.getConfigurator().apply(cfg);
    }

    public CommandBase manualControlCommand(
            DoubleSupplier turretControl, DoubleSupplier fourBarControl) {
        return new RunCommand(
                        () -> {
                            manualControl(turretControl.getAsDouble(), fourBarControl.getAsDouble());
                        },
                        this)
                .ignoringDisable(true);
    }

    public void manualControl(double turretControl, double fourBarControl) {
        m_turretController.setControl(m_turretOut.withOutput(turretControl * 0.1));
        m_fourBarController.setControl(m_fourBarOut.withOutput(fourBarControl * 0.2));

        System.out.println("Four bar is driving at " + fourBarControl);
    }
}
