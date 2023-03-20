package frc.robot.commands;

import frc.robot.Constants.Turn180PID;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pulley;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turn180 extends CommandBase {
  private final Drive m_drive;
  private final Pulley m_pulley;
  private PIDController m_pid;
  private double m_setPoint;
  private boolean m_finished;
  private final Timer m_timer = new Timer();

  public Turn180(Drive drive, Pulley pulley) {
    m_drive = drive;
    m_pulley = pulley;
    //m_pid = new PIDController(Turn180PID.kP, Turn180PID.kI, Turn180PID.kD);
    //m_pid.setTolerance(1.0);
    SmartDashboard.putNumber("TurnKp", 0);
    SmartDashboard.putNumber("TurnKi", 0);
    SmartDashboard.putNumber("TurnKd", 0);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    double kp = SmartDashboard.getNumber("TurnKp", 0);
    double ki = SmartDashboard.getNumber("TurnKi", 0);
    double kd = SmartDashboard.getNumber("TurnKd", 0);
    m_pid = new PIDController(kp, ki, kd);
    m_pid.setTolerance(2.0, 5.0);
    m_pid.setSetpoint(0);
    m_drive.resetGyro();
    m_setPoint = m_drive.getAngle() - 180.0;
    m_finished = false;

    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double d = (m_drive.getAngle() - m_setPoint);
    m_drive.curvatureDrive(0, m_pid.calculate(d));
    if(m_pid.atSetpoint()){
      m_finished = true;
    }
    m_pulley.slowClosePulley();
    SmartDashboard.putNumber("Turn 180 sec", m_timer.get());
    SmartDashboard.putNumber("dist", d);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
