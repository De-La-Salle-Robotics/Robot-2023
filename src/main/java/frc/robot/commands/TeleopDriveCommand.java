package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private DoubleSupplier m_leftJoystick;
    private DoubleSupplier m_rightJoystick;

    private double speedControl;
    private BooleanSupplier m_buttonBack;
    private BooleanSupplier m_buttonStart;

    private DriveBaseSubsystem m_DriveBaseSubsystem;

    public TeleopDriveCommand(
            DoubleSupplier leftJoystick,
            DoubleSupplier rightJoystick,
            BooleanSupplier buttonBack,
            BooleanSupplier buttonStart,
            DriveBaseSubsystem driveBaseSubsystem) {

        speedControl = 0.25;
        m_buttonBack = buttonBack;
        m_buttonStart = buttonStart;
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;

        m_DriveBaseSubsystem = driveBaseSubsystem;
        addRequirements(m_DriveBaseSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveBaseSubsystem.chassisControl(0.0, 0.0, 0.0);
    }

    @Override
    public void execute() {

        if (m_buttonBack.getAsBoolean()) {
            speedControl = 0.1;
        }
        if (m_buttonStart.getAsBoolean()) {
            speedControl = 0.25;
        }

        // The left joystick is negative due to the controller being in pilot/flight mode
        m_DriveBaseSubsystem.chassisControl(
                m_leftJoystick.getAsDouble(), m_rightJoystick.getAsDouble(), speedControl);
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveBaseSubsystem.chassisControl(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
