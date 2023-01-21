package frc.pilotlib.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public class OverrideableCommand extends CommandBase {
    public static class TriggerCommandPair {
        BooleanSupplier m_trigger;
        Command m_command;

        public TriggerCommandPair(BooleanSupplier trigger, Command command) {
            m_trigger = trigger;
            m_command = command;
        }
    }

    private boolean m_isFinished;

    TriggerCommandPair[] m_triggerCommandPairs;
    Command m_lastCommand = null;
    Command m_defaultCommand;
    /**
     * List of trigger command pairs, order matters with priority going to the first ones
     *
     * @param commandPairs
     */
    public OverrideableCommand(
            Command defaultCommand, Subsystem requirement, TriggerCommandPair... commandPairs) {
        m_triggerCommandPairs = commandPairs;
        m_defaultCommand = defaultCommand;
        m_isFinished = false;

        addRequirements(requirement);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        for (TriggerCommandPair pair : m_triggerCommandPairs) {
            if (pair.m_trigger.getAsBoolean()) {
                /* If this trigger is true, use this command */
                if (pair.m_command != m_lastCommand) {
                    /* If this command wasn't called last, then we need to end and initialize */
                    if (m_lastCommand != null) {
                        /* End the old command if it's not null */
                        m_lastCommand.end(true);
                    }
                    /* Initialize the new command */
                    pair.m_command.initialize();
                }
                /* Execute the new command */
                pair.m_command.execute();
                /* Save this command in case it changes */
                m_lastCommand = pair.m_command;
                return;
            }
        }
        /* If this trigger is true, use this command */
        if (m_defaultCommand != m_lastCommand) {
            /* If this command wasn't called last, then we need to end and initialize */
            if (m_lastCommand != null) {
                /* End the old command if it's not null */
                m_lastCommand.end(true);
            }
            /* Initialize the new command */
            m_defaultCommand.initialize();
        }
        /* We need to execute our default command */
        m_defaultCommand.execute();
        m_lastCommand = m_defaultCommand;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
