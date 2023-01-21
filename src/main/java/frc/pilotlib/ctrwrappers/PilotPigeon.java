package frc.pilotlib.ctrwrappers;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PilotPigeon extends WPI_Pigeon2 {
    public PilotPigeon(int deviceNumber) {
        super(deviceNumber);
    }

    public PilotPigeon(int deviceNumber, String canbusName) {
        super(deviceNumber, canbusName);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
