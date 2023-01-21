// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.pilotlib.controllerwrappers.DriverController;
import frc.pilotlib.controllerwrappers.DriverController.Axis;
import frc.pilotlib.controllerwrappers.DriverController.Button;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drivetrain CAN IDs
    public static final int Left_Leader_ID = 0;
    public static final int Left_Middle_ID = 1;
    public static final int Left_Back_ID = 2;

    public static final int Right_Leader_ID = 3;
    public static final int Right_Middle_ID = 4;
    public static final int Right_Back_ID = 5;

    // Controls
    public static class DriverConstants {
        // Driver Controller
        public static final int Driver_Controller_Port = 0;
        private static final DriverController m_driverController =
                new DriverController(Driver_Controller_Port);

        public static final DoubleSupplier leftJoyStickY = m_driverController.getAxis(Axis.kLeftY);
        public static final DoubleSupplier rightJoyStickX = m_driverController.getAxis(Axis.kRightX);

        public static final BooleanSupplier leftBumper =
                m_driverController.getButtonSupplier(Button.kLeftBumper);
        public static final BooleanSupplier rightBumper =
                m_driverController.getButtonSupplier(Button.kRightBumper);

        public static final BooleanSupplier buttonStart =
                m_driverController.getButtonSupplier(Button.kStart);
        public static final BooleanSupplier buttonBack =
                m_driverController.getButtonSupplier(Button.kBack);

        public static final BooleanSupplier button_A = m_driverController.getButtonSupplier(Button.kA);
        public static final BooleanSupplier button_Y = m_driverController.getButtonSupplier(Button.kY);
    }
}
