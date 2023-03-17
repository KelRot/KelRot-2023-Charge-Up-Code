package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class ConeSecondNode extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Timer m_timer = new Timer();
  private boolean m_finished = false;

  public ConeSecondNode(Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_finished = false;

    m_pulley.set(PulleyConstants.kArmOpenStateLength);

    SmartDashboard.putBoolean("Cone Second", true);
  }

  @Override
  public void execute() {
    if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength){
      m_pneumatics.getArmSolenoid().open();
      m_timer.start();
    }
    if(m_timer.get() >= 1.0){
      m_pneumatics.getIntakeSolenoid().open();
    }
    if(m_timer.get() >= 1.3){
      m_pneumatics.getArmSolenoid().close();
      m_pulley.set(PulleyConstants.kFullCloseStateLength);
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
