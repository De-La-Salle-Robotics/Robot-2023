package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenixpro.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Function;

public class CtrUtils {

    public static void runUntilSuccessWithTimeoutV5(
            Function<Integer, ErrorCode> funcToRun, int methodTimeout, int retryCount) {
        ErrorCode retval;
        do {
            retval = funcToRun.apply(methodTimeout);
        } while (retval != ErrorCode.OK && --retryCount > 0);

        if (retryCount == 0) {
            DriverStation.reportError(
                    "Could not successfully run function: " + retval, Thread.currentThread().getStackTrace());
        }
    }

    public static void runUntilSuccessWithTimeoutPro(
            Function<Double, StatusCode> funcToRun, double methodTimeout, int retryCount) {
        StatusCode retval;
        do {
            retval = funcToRun.apply(methodTimeout);
        } while (!retval.isOK() && --retryCount > 0);

        if (retryCount == 0) {
            DriverStation.reportError(
                    "Could not successfully run function: " + retval, Thread.currentThread().getStackTrace());
        }
    }
}
