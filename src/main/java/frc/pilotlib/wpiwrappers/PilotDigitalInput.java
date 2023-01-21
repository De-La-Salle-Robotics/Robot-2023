package frc.pilotlib.wpiwrappers;

import edu.wpi.first.wpilibj.DigitalInput;

public class PilotDigitalInput extends DigitalInput {
    private boolean m_isInverted;

    public PilotDigitalInput(int channel, boolean invert) {
        super(channel);
        m_isInverted = invert;
    }

    @Override
    public boolean get() {
        return super.get() ^ m_isInverted;
    }
}
