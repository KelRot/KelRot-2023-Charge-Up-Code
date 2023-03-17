package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class AutoScore extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Timer m_timer = new Timer(), m_autoTimer = new Timer();
  private boolean m_finished = false, m_closing;

  public AutoScore(Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_autoTimer.reset();
    m_autoTimer.start();
    
    m_finished = false;
    m_closing = false;

    m_pulley.reset();
    m_pulley.set(PulleyConstants.kFullOpenStateLength);
  }

  @Override
  public void execute() {
    if(!m_closing){
      if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength){
        m_pneumatics.getArmSolenoid().open();
      }
      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - PulleyConstants.kMomentumTolerance){
        m_pneumatics.getTelescopeSolenoid().open();
        m_pneumatics.getIntakeSolenoid().open();
        m_timer.start();
      }
      if(m_timer.get() >= 0.2){
        m_pneumatics.getTelescopeSolenoid().close();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
        m_closing = true;
      }
    }else{
      if(m_pulley.getDistance() <= PulleyConstants.kOnGroundStateLength){
        m_pneumatics.getArmSolenoid().close();
        m_finished = true;
      }
    }
    SmartDashboard.putNumber("Auto Time", m_autoTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_pulley.reset();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
