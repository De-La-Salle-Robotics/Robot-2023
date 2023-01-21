package frc.pilotlib.ctrwrappers;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PilotCoder extends WPI_CANCoder {
    public PilotCoder(int deviceNumber) {
        super(deviceNumber);
    }

    public PilotCoder(int deviceNumber, String canbusName) {
        super(deviceNumber, canbusName);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", this::getAbsolutePosition, null);
    }
}
