package frc.pilotlib.controllerwrappers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** 7762's Operator Controller is a logitech F310 in D-input mode. */
public class OperatorController extends GenericHID {

    /** Represents a digital button. */
    public enum Button {
        kLeftBumper(5),
        kRightBumper(6),
        kLeftTrigger(7),
        kRightTrigger(8),
        kLeftStick(11),
        kRightStick(12),
        kA(2),
        kB(3),
        kX(1),
        kY(4),
        kBack(9),
        kStart(10);

        @SuppressWarnings("MemberName")
        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    /** Represents an axis. */
    public enum Axis {
        kLeftX(0),
        kRightX(2),
        kLeftY(1),
        kRightY(3);

        @SuppressWarnings("MemberName")
        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum POV {
        kUp(0),
        kRight(90),
        kDown(180),
        kLeft(270);

        @SuppressWarnings("MemberName")
        public final int value;

        POV(int value) {
            this.value = value;
        }
    }

    public OperatorController(int port) {
        super(port);
        /* Verify this matches the logitech controller */
    }

    public DoubleSupplier getAxis(Axis axis) {
        return () -> getRawAxis(axis.value);
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(this, button.value);
    }

    public POVButton getButton(POV pov) {
        return new POVButton(this, pov.value);
    }

    public BooleanSupplier getThreshold(Axis axis, double value, boolean isAbs) {
        return () -> {
            double ax = getAxis(axis).getAsDouble();
            if (isAbs) {
                ax = Math.abs(ax);
            }
            return ax > value;
        };
    }

    public BooleanSupplier getButtonSupplier(Button button) {
        return (new JoystickButton(this, button.value));
    }
}
