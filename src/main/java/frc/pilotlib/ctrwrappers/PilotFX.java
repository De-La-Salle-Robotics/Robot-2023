package frc.pilotlib.ctrwrappers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Wrapper class to provide reasonable values for TalonFX */
public class PilotFX extends WPI_TalonFX {
    public static final double kUnitsPerRotation = 2048.0; // 2048 units per rotation
    public static final double k100msPerMinute = 600.0; // 10 100ms per second, 60 seconds per minute

    public PilotFX(int deviceNumber) {
        super(deviceNumber);
    }

    public PilotFX(int deviceNumber, String canbusName) {
        super(deviceNumber, canbusName);
    }

    public static double toRotations(double rawUnits) {
        return rawUnits / kUnitsPerRotation;
    }

    public static int toRawUnits(double rotations) {
        return (int) (rotations * kUnitsPerRotation);
    }

    public static double toRPM(double rawVelUnits) {
        return rawVelUnits * k100msPerMinute / kUnitsPerRotation;
    }

    public static int toRawVelUnits(double rpm) {
        return (int) (rpm * kUnitsPerRotation / k100msPerMinute);
    }

    public double getRotations() {
        return toRotations(getSelectedSensorPosition());
    }

    public void setPosition(double rotations) {
        setSelectedSensorPosition(toRawUnits(rotations));
    }

    public double getRPM() {
        return toRPM(getSelectedSensorVelocity());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Position", this::getRotations, this::setPosition);
        builder.addDoubleProperty("Velocity", this::getRPM, null);
    }
}
