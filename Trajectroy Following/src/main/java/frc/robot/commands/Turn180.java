package frc.robot.commands;

import frc.robot.PIDDebugger;
import frc.robot.Constants.Turn180PID;
import frc.robot.paths.P;
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
  private final PIDDebugger m_pidDebugger;
  private double m_setPoint;
  private boolean m_finished;
  private final Timer m_timer = new Timer();

  public Turn180(Drive drive, Pulley pulley) {
    m_drive = drive;
    m_pulley = pulley;
    m_pidDebugger = new PIDDebugger();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.stopMotors();
    m_drive.setIdleModeBrake(true);
    m_drive.setFastMode();

    m_pid = m_pidDebugger.getPIDControllerFromDashboard("Turn");
    m_pid.setTolerance(2.0, 5.0);
    m_pid.setSetpoint(180.0);

    m_drive.resetOdometry(m_drive.getPose());

    m_timer.reset();
    m_timer.start();

    m_finished = false;
  }

  @Override
  public void execute() {
    double leftVolts, rightVolts;
    rightVolts = m_pid.calculate(-m_drive.getAngle());
    leftVolts = -rightVolts;

    if(m_pid.atSetpoint()){
      m_finished = true;
    }
    
    m_drive.tankDriveVolts(leftVolts, rightVolts);
    m_pulley.slowClosePulley();

    SmartDashboard.putBoolean("Turn 180 Is Finished", m_finished);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setNormalMode();
    m_drive.setIdleModeBrake(false);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
