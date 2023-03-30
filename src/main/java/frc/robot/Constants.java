// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String kCANivoreCANbus = "canivore";

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class DriveTrainConstants {
        public static final int kRghtDriveId1 = 0;
        public static final int kRghtDriveId2 = 1;
        public static final int kRghtDriveId3 = 2;
        public static final int kLeftDriveId1 = 3;
        public static final int kLeftDriveId2 = 4;
        public static final int kLeftDriveId3 = 5;
    }

    public static class ClawConstants {
        public static final int kClawTalonId = 20;
    }

    public static class ElevatorConstants {
        public static final int kElevatorTalonId = 10;
        public static final int kExtenderTalonId = 11;

        public static final int kExtenderForwardLimitPort = 0;
        public static final int kExtenderReverseLimitPort = 1;
    }
}
