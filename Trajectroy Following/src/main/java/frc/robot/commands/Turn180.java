package frc.robot.commands;

import frc.robot.Constants.Turn180PID;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turn180 extends CommandBase {
  private final Drive m_drive;
  private final PIDController m_pid;
  private double m_setPoint;
  private boolean m_finished;

  public Turn180(Drive drive) {
    m_drive = drive;
    m_pid = new PIDController(Turn180PID.kP, Turn180PID.kI, Turn180PID.kD);
    m_pid.setTolerance(1.0);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_setPoint = m_drive.getAngle() - 180.0;
    m_finished = false;
  }

  @Override
  public void execute() {
    double d = m_drive.getAngle() - m_setPoint;
    m_drive.curvatureDrive(0, m_pid.calculate(d));
    if(m_pid.atSetpoint()){
      m_finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
