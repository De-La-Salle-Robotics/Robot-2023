package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static CANdle m_CANdle = new CANdle(20);

    public void init_setep() {
        m_CANdle.clearAnimation(0);
        m_CANdle.setLEDs(255, 0, 255);
    }

    public void SetLED(int r, int g, int b) {
        m_CANdle.setLEDs(r, g, b);
    }
}
