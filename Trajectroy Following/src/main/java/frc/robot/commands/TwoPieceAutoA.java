package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.Constants.Turn180PID;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class TwoPieceAutoA extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Drive m_drive;
  private final PIDController m_pid;
  private final Timer m_timer = new Timer();
  private boolean m_finished = false, m_closing, m_turned, m_turning;
  private double m_setPoint;

  public TwoPieceAutoA(Pneumatics pneumatics, Pulley pulley, Drive drive) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;

    m_pid = new PIDController(Turn180PID.kP, Turn180PID.kI, Turn180PID.kD);
    m_pid.setTolerance(1.0);
    
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_finished = false;
    m_closing = false;
    m_turning = false;

    m_setPoint = m_drive.getAngle() - 180.0;

    m_pulley.set(PulleyConstants.kFullOpenStateLength);

    SmartDashboard.putBoolean("Cube Third", true);
  }

  @Override
  public void execute() {
    if(!m_closing){
      if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength){
        m_pneumatics.getArmSolenoid().open();
      }
      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - PulleyConstants.kTolerance){
        m_pneumatics.getTelescopeSolenoid().open();
        m_timer.start();
      }
      if(m_timer.get() >= 1.0){
        m_pneumatics.getIntakeSolenoid().open();
      }
      if(m_timer.get() >= 1.2){
        m_pneumatics.getTelescopeSolenoid().close();
        m_pulley.set(PulleyConstants.kOnGroundStateLength);
        m_closing = true;
      }
    }else{
      if(m_pulley.getDistance() <= PulleyConstants.kOnGroundStateLength){
        m_pneumatics.getArmSolenoid().close();
      }
      if(m_turning == false){
        m_drive.tankDriveVolts(-6, -6);
      }else{
        m_drive.curvatureDrive(0, m_pid.calculate(m_drive.getAngle() - m_setPoint));
        if(m_pid.atSetpoint())
          m_finished = true;
      }
      if(m_drive.getDistance()[0] >= 3.0)
        m_turning = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
