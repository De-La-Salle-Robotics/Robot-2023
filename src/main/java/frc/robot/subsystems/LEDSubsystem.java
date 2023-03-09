package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {    
    private CANdle m_candle = new CANdle(6, "canivore");

    private int RED = 125;
    private int GREEN = 0;
    private int BLUE = 125;

    private boolean pgToggle = true;
    private int wait = 0;
    private boolean fadeToPurple = true;
    private boolean fadeToGold = false;
    private boolean hold = false;

    public void startup() {
        // Initial setup
        m_candle.clearAnimation(0);
        m_candle.setLEDs(0, 0, 0);
    }

    public void PurpleGold() {
        if (!pgToggle) return;

        if (hold) {
            wait++;
            if (fadeToPurple & wait > 100) {
                wait = 0;
                hold = false;
                fadeToPurple = false;
                fadeToGold = true;
                return;
            }
            if (fadeToGold & wait > 100) {
                wait = 0;
                hold = false;
                fadeToPurple = true;
                fadeToGold = false;
                return;
            }
            return;
        }

        if (fadeToPurple) {
            if (RED > 40) {
                RED -= 3;
                if (RED < 40) {
                    RED = 40;
                }
            }
            if (GREEN > 0) {
                GREEN -= 2;
                if (GREEN < 0) {
                    GREEN = 0;
                }
            }
            if (BLUE < 56) {
                BLUE++;
            }

            if (RED == 40 & GREEN == 0 & BLUE == 56) {
                hold = true;
            }
            m_candle.setLEDs(RED, GREEN, BLUE);
        }

        if (fadeToGold) {
            if (RED < 255) {
                RED++;
            }
            if (GREEN < 180) {
                GREEN++;
            }
            if (BLUE > 0) {
                BLUE--;
            }

            if (RED == 255 & GREEN == 180 & BLUE == 0) {
                hold = true;
            }
            m_candle.setLEDs(RED, GREEN, BLUE);
        }
        // if (wait > 0 & wait < 50){
        //    m_CANdle_0.setLEDs(40, 0, 56);
        // }
        // if (wait > 50){
        //    m_CANdle_0.setLEDs(255, 180, 0);
        // }
    }

    // Set LEDs to specific values --- most likely to be used for status
    public void setLED0(int R, int G, int B, int startIndex, int count) {
        m_candle.setLEDs(RED, GREEN, BLUE, 0, startIndex, count);
    }
}