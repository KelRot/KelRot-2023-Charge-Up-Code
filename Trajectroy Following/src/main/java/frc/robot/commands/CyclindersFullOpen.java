package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class CyclindersFullOpen extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Timer m_timer = new Timer();
  private boolean m_finished = false;

  public CyclindersFullOpen(Pneumatics pneumatics) {
    m_pneumatics = pneumatics;
    addRequirements(m_pneumatics);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_finished = false;
    m_pneumatics.getArmSolenoid().open();
  }

  @Override
  public void execute() {
    if(m_timer.get() >= 0.9){
      m_pneumatics.getTelescopeSolenoid().open();
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
