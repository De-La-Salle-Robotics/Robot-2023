package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.TimedRobot;

public class LEDSubsystem extends TimedRobot {

    private static CANdle m_CANdle = new CANdle(6, "canivore");

    private void ledSetup() {
        m_CANdle.configBrightnessScalar(0.5, 0);
        //m_CANdle.setLEDs(255, 0, 0, 50, 0, 4);
        m_CANdle.setLEDs(255, 0, 255);
    }

    public void SetLED(int r, int g, int b) {
        m_CANdle.setLEDs(r, g, b);
    }

    @Override
    public void robotInit(){
        ledSetup();
    }
}