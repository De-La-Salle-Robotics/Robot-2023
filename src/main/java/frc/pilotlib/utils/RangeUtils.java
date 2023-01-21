package frc.pilotlib.utils;

public class RangeUtils {
    public static boolean isInRange(double value, double target, double threshold) {
        return Math.abs(value - target) < threshold;
    }
}
