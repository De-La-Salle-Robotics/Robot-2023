// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EffectorState;
import frc.robot.subsystems.EndEffectorSubsystem.VaccuumState;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandPS4Controller m_operatorController =
            new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(
                m_driveSubsystem.arcadeDriveCommand(
                        m_driverController::getLeftY, m_driverController::getRightX));
        m_turretSubsystem.setDefaultCommand(
                m_turretSubsystem.manualControlCommand(
                        m_operatorController::getLeftX, m_operatorController::getRightY));

        Supplier<EndEffectorSubsystem.VaccuumState> vaccuumStateSupplier =
                () -> {
                    switch (m_operatorController.getHID().getPOV()) {
                        default:
                            return VaccuumState.NoChange;
                        case 90:
                            return VaccuumState.Released;
                        case 180:
                            return VaccuumState.Stopped;
                        case 270:
                            return VaccuumState.Sucking;
                    }
                };
        Supplier<EndEffectorSubsystem.EffectorState> effectorStateSupplier =
                () -> {
                    if (m_operatorController.triangle().getAsBoolean()) {
                        return EffectorState.Up;
                    }
                    if (m_operatorController.cross().getAsBoolean()) {
                        return EffectorState.Down;
                    }
                    return EffectorState.NoChange;
                };
        m_operatorController
                .L1()
                .whileTrue(
                        m_endEffectorSubsystem.controlCubeSide(vaccuumStateSupplier, effectorStateSupplier));
        m_operatorController
                .R1()
                .whileTrue(
                        m_endEffectorSubsystem.controlConeSide(vaccuumStateSupplier, effectorStateSupplier));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.driveStraight(m_driveSubsystem, 0.1);
    }
}
