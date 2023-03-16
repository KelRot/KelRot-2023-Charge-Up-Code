package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargingConstants;
import frc.robot.subsystems.Drive;

public class ChargingStation extends CommandBase {
  private final Drive m_drive;
  private final PIDController m_pid;
  private boolean m_isReached;
  private boolean m_finished;
  private double m_prevMaxOutput;

  public ChargingStation(Drive drive) {
    m_drive = drive;

    m_pid = new PIDController(ChargingConstants.kP, ChargingConstants.kI, ChargingConstants.kD);
    m_pid.setTolerance(6, 5);

    m_finished = false;
    m_isReached = false;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Charging State", "Initialized");

    m_prevMaxOutput = m_drive.getMaxOutput();
    m_isReached = false;
    m_drive.setNormalMode();
    m_drive.setGyroAxis(IMUAxis.kY);
    m_drive.resetGyro();
    m_pid.reset();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Charging State", "Executing");
    
    SmartDashboard.putBoolean("Is Reached", m_isReached);

    if(m_isReached == false) {
      m_drive.tankDriveVolts(-7, -7);
      if(Math.abs(m_drive.getAngle()) > ChargingConstants.kRequiredAngle)
        m_isReached = true;
    }
    else {
      double volts = m_pid.calculate(m_drive.getAngle(), 0);
      m_drive.tankDriveVolts(volts, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Charging State", "Finished");
    
    m_drive.setGyroAxis(IMUAxis.kZ);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}