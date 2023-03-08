package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveTrainConstants.*;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class DriveSubsystem extends SubsystemBase {

    private TalonFX m_leftDrive1 = new TalonFX(kLeftDriveId1, kCANivoreCANbus);
    private TalonFX m_leftDrive2 = new TalonFX(kLeftDriveId2, kCANivoreCANbus);
    private TalonFX m_leftDrive3 = new TalonFX(kLeftDriveId3, kCANivoreCANbus);
    private TalonFX m_rghtDrive1 = new TalonFX(kRghtDriveId1, kCANivoreCANbus);
    private TalonFX m_rghtDrive2 = new TalonFX(kRghtDriveId2, kCANivoreCANbus);
    private TalonFX m_rghtDrive3 = new TalonFX(kRghtDriveId3, kCANivoreCANbus);

    private DutyCycleOut m_leftOut = new DutyCycleOut(0);
    private DutyCycleOut m_rghtOut = new DutyCycleOut(0);

    private Follower m_leftFollow = new Follower(m_leftDrive1.getDeviceID(), false);
    private Follower m_rghtFollow = new Follower(m_rghtDrive1.getDeviceID(), false);

    public DriveSubsystem() {
        m_leftDrive2.setControl(m_leftFollow);
        m_leftDrive3.setControl(m_leftFollow);
        m_rghtDrive2.setControl(m_rghtFollow);
        m_rghtDrive3.setControl(m_rghtFollow);
    }

    public CommandBase arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier turn, BooleanSupplier slowdownEnabled, BooleanSupplier speedupEnabled) {
        return new RunCommand(
                        () -> {
                            arcadeDrive(forward.getAsDouble(), turn.getAsDouble(), slowdownEnabled.getAsBoolean(), speedupEnabled.getAsBoolean());
                        },
                        this)
                .ignoringDisable(true);
    }

    public void arcadeDrive(double forward, double turn, boolean slowdownEnabled, boolean speedupEnabled) {
        double speed = 0.3;
        if (slowdownEnabled) {
            speed = 0.15;
        }
        if (speedupEnabled) {
            speed = 0.45;
        }

        double inverseSpeed = 1 - speed;
        inverseSpeed /= 1.5;
        turn *= (1 - inverseSpeed);
        forward *= speed;

        m_leftDrive1.setControl(m_leftOut.withOutput(-(forward - turn)));
        //0.9 for slight left drift error
        m_rghtDrive1.setControl(m_rghtOut.withOutput((forward + turn) * 0.9));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
