package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

public class LEDCommand extends CommandBase {

    private LEDSubsystem m_LEDSubsystem;
    private BooleanSupplier m_Y;
    private BooleanSupplier m_A;
    private BooleanSupplier m_isDisabled;

    public LEDCommand(
            BooleanSupplier A, BooleanSupplier Y, BooleanSupplier isDisabled, LEDSubsystem ledSubsystem) {
        m_Y = Y;
        m_A = A;
        m_isDisabled = isDisabled;

        m_LEDSubsystem = ledSubsystem;
        addRequirements(m_LEDSubsystem);
    }

    @Override
    public void execute() {
        if (m_isDisabled.getAsBoolean()) {
            m_LEDSubsystem.isDisabled();
        } else {
            m_LEDSubsystem.PurpleGold();
            m_LEDSubsystem.updateLED0(m_A.getAsBoolean(), m_Y.getAsBoolean());
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
