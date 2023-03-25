package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX m_elevatorMotor = new TalonFX(kElevatorTalonId, kCANivoreCANbus);
    private final TalonFX m_extenderMotor = new TalonFX(kExtenderTalonId, kCANivoreCANbus);

    private final DutyCycleOut m_elevatorPower = new DutyCycleOut(0);
    private final DutyCycleOut m_extenderPower = new DutyCycleOut(0);

    public ElevatorSubsystem() {}

    public CommandBase manualControlElevatorCommand(
            DoubleSupplier elevatorPower, DoubleSupplier extenderPower) {
        return new RunCommand(
                        () -> {
                            manualControlElevator(elevatorPower.getAsDouble(), extenderPower.getAsDouble());
                        },
                        this)
                .ignoringDisable(true);
    }

    public void manualControlElevator(double elevatorPower, double extenderPower) {
        m_elevatorMotor.setControl(m_elevatorPower.withOutput(elevatorPower));
        m_extenderMotor.setControl(m_extenderPower.withOutput(extenderPower));
    }
}