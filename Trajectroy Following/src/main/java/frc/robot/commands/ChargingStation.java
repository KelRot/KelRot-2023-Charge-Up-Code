package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargingConstants;
import frc.robot.subsystems.Drive;

public class ChargingStation extends CommandBase {
  private final Drive m_drive;
  private final PIDController m_pid;
  private boolean m_finished;
  private double m_prevMaxOutput;

  public ChargingStation(Drive drive) {
    m_drive = drive;

    m_pid = new PIDController(ChargingConstants.kP, ChargingConstants.kI, ChargingConstants.kD);
    m_pid.setTolerance(3, 5);
    //m_pid.setIntegratorRange(-0.5, 0.5);

    m_finished = false;
    m_prevMaxOutput = 8.0;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_prevMaxOutput = m_drive.getMaxOutput();
    m_drive.setMaxOutput(4);
    m_pid.reset();
  }

  @Override
  public void execute() {
    double volts = m_pid.calculate(m_drive.getAngle(), 0);
    m_drive.tankDriveVolts(volts, volts);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxOutput(m_prevMaxOutput);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}