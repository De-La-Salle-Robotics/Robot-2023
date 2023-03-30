// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {
    public static CommandBase driveStraight(DriveSubsystem drive, double timeToDrive) {
        return drive
                .arcadeDriveCommand(
                        () -> {
                            return -0.4;
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
                                }));
    }

    public static CommandBase doNothing(DriveSubsystem drive) {
        return drive.arcadeDriveCommand(
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
                });
    }

    public static CommandBase spitAndDriveBack(DriveSubsystem drive, ClawSubsystem claw) {
        return new RunCommand(
                        () -> drive.arcadeDrive(1, 0, false, false),
                        drive) // Drive fast backwards to knock claw down
                .withTimeout(0.5)
                .andThen(
                        new RunCommand(() -> drive.arcadeDrive(0, 0, false, false), drive)
                                .withTimeout(0.3)) // Stop driving to allow claw to fall
                .andThen(
                        new RunCommand(() -> drive.arcadeDrive(-0.2, 0, false, false), drive)
                                .withTimeout(1.8)) // Drive forward to make up distance from driving backward
                .andThen(new InstantCommand(() -> claw.blow(), claw)) // Blow
                .andThen(
                        new RunCommand(() -> drive.arcadeDrive(0.2, 0, false, false), drive)
                                .withTimeout(1)) // And start driving backwards
                .andThen(new InstantCommand(() -> claw.neutral(), claw)) // Stop blowing
                .andThen(
                        new RunCommand(() -> drive.arcadeDrive(0.2, 0, false, false), drive)
                                .withTimeout(5)) // And continue to drive backwards
                .andThen(
                        new InstantCommand(
                                () -> drive.arcadeDrive(0, 0, false, false), drive)); // Until we cross the line
    }

    public enum AutoRoutines {
        DoNothing,
        DriveForward,
        SpitAndDrive
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
