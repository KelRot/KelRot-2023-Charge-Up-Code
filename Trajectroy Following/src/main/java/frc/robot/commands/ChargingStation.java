package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDDebugger;
import frc.robot.Constants.ChargingConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pulley;

public class ChargingStation extends CommandBase {
  private final Drive m_drive;
  private final Pulley m_pulley;
  private PIDController m_pid;
  private final PIDDebugger m_pidDebugger;
  private boolean m_isReached;
  private boolean m_finished;

  public ChargingStation(Drive drive, Pulley pulley) {
    m_drive = drive;
    m_pulley = pulley;
    m_pidDebugger = new PIDDebugger();
    m_pid = new PIDController(0, 0, 0);
    
    m_finished = false;
    m_isReached = false;

    addRequirements(m_drive, m_pulley);

  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Charging State", "Initialized");

    m_pid = m_pidDebugger.getPIDControllerFromDashboard("Charging");
    m_pid.setTolerance(5, 3.0);
    m_pid.setSetpoint(0);

    m_finished = false;
    m_isReached = false;
    m_drive.setFastMode();
    m_drive.setIdleModeBrake(false);
    m_drive.setGyroAxis(IMUAxis.kY);
    m_drive.resetGyro();
    m_pid.reset();
  }

  @Override
  public void execute() {
    double volts;
    SmartDashboard.putString("Charging State", "Executing");
    
    SmartDashboard.putBoolean("Is Reached", m_isReached);
    SmartDashboard.putBoolean("Charging At Setpoint", m_pid.atSetpoint());

    if(m_isReached == false) {
      volts = -6.5;
      if(Math.abs(m_drive.getAngle()) > ChargingConstants.kRequiredAngle) {
        m_isReached = true;
        m_drive.setIdleModeBrake(true);
      }
    }
    else {
      volts = -m_pid.calculate(m_drive.getAngle(), 0);
    }
    if(m_pid.atSetpoint()) {
      m_finished = true;
      volts = 0.0;
    }

    m_pulley.fixedPulley();
    m_drive.tankDriveVolts(volts, volts);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Charging State", "Finished");
    m_drive.setGyroAxis(IMUAxis.kZ);
    m_drive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}