package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargingConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pulley;

public class ChargingStation extends CommandBase {
  private final Drive m_drive;
  private final Pulley m_pulley;
  private final PIDController m_pid;
  private boolean m_isReached;
  private boolean m_finished;

  public ChargingStation(Drive drive, Pulley pulley) {
    m_drive = drive;
    m_pulley = pulley;

    m_pid = new PIDController(ChargingConstants.kP, ChargingConstants.kI, ChargingConstants.kD);
    m_pid.setTolerance(5, 2.5);
    m_pid.setSetpoint(0);

    m_finished = false;
    m_isReached = false;

    addRequirements(m_drive);
    addRequirements(m_pulley);

  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Charging State", "Initialized");

    m_isReached = false;
    m_drive.setFastMode();
    m_drive.setIdleModeBrake(true);
    m_drive.setMaxOutput(8);
    m_drive.setGyroAxis(IMUAxis.kY);
    m_drive.resetGyro();
    m_pid.reset();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Charging State", "Executing");
    
    SmartDashboard.putBoolean("Is Reached", m_isReached);

    if(m_isReached == false) {
      m_drive.tankDriveVolts(ChargingConstants.kRequiredVoltageBack, ChargingConstants.kRequiredVoltageBack); // goes forward
      if(Math.abs(m_drive.getAngle()) > ChargingConstants.kRequiredAngle)
        m_isReached = true;
    }
    else {
      double volts = -m_pid.calculate(m_drive.getAngle(), 0);
      m_drive.tankDriveVolts(volts, volts);
    }
    m_pulley.slowClosePulley();

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