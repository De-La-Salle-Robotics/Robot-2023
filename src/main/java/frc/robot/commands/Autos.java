// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {
    public static CommandBase driveStraight(DriveSubsystem drive, double timeToDrive) {
        return drive
                .arcadeDriveCommand(
                        () -> {
                            return 0.1;
                        },
                        () -> {
                            return 0;
                        },
                        () -> {
                            return false;
                        },
                        () -> {
                            return false;
                        })
                .withTimeout(timeToDrive)
                .andThen(
                        drive.arcadeDriveCommand(
                                () -> {
                                    return 0;
                                },
                                () -> {
                                    return 0;
                                },
                                () -> {
                                    return false;
                                },
                                () -> {
                                    return false;
                                }
                                ));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
