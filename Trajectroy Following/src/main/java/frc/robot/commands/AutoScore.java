package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class AutoScore extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Timer m_timer = new Timer();
  private boolean m_finished;

  public AutoScore(Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_pneumatics.getArmSolenoid().open();
    m_pulley.openState();
    m_timer.reset();
    m_timer.start();
    m_finished = false;
  }

  @Override
  public void execute() {
    if(m_pneumatics.getTelescopeSolenoid().getState() == false){
      if(m_timer.get() >= PulleyConstants.kTime){
        m_pneumatics.getTelescopeSolenoid().open();
        m_timer.reset();
      }
    }else{
      if(m_timer.get() >= PulleyConstants.kTelescopeToIntake){
        m_pneumatics.getIntakeSolenoid().open();
        m_finished = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
