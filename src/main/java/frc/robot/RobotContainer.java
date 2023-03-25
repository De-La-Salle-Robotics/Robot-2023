// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.DesiredLocation;
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
    private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

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
                        m_driverController::getLeftY,
                        m_driverController::getRightX,
                        m_driverController.leftBumper(),
                        m_driverController.rightBumper()));

        Supplier<DesiredLocation> elevatorLocationSupplier =
                () -> {
                    var controller = m_operatorController.getHID();
                    if (controller.getAButton()) return DesiredLocation.Stow;
                    if (controller.getXButton()) return DesiredLocation.MiddleScore;
                    if (controller.getYButton()) return DesiredLocation.HighScore;
                    if (controller.getBButton()) return DesiredLocation.LoadStationPickup;
                    return DesiredLocation.NoChange;
                };
        m_elevatorSubsystem.setDefaultCommand(
                m_elevatorSubsystem.setClosedLoopCommand(elevatorLocationSupplier));

        new Trigger(
                        /* If the joystick is deflected, fall back to manual control */
                        () -> {
                            return Math.abs(m_operatorController.getLeftY()) > 0.1
                                    || Math.abs(m_operatorController.getRightY()) > 0.1;
                        })
                .whileTrue(
                        m_elevatorSubsystem.manualControlElevatorCommand(
                                () -> {
                                    return -m_operatorController.getLeftY();
                                },
                                () -> {
                                    return -m_operatorController.getRightY();
                                }));

        Supplier<ClawSubsystem.ClawState> clawStateSupplier =
                () -> {
                    switch (m_operatorController.getHID().getPOV()) {
                        default:
                            return ClawState.Neutral;
                        case 0:
                            return ClawState.Blowing;
                        case 180:
                            return ClawState.Sucking;
                    }
                };
        m_clawSubsystem.setDefaultCommand(m_clawSubsystem.controlClaw(clawStateSupplier));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.driveStraight(m_driveSubsystem, 1.25);
    }
}
