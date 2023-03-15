package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class CyclindersFullClose extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;

  public CyclindersFullClose(Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_pneumatics.getTelescopeSolenoid().close();
    m_pneumatics.getArmSolenoid().close();
    m_pulley.set(PulleyConstants.kHalfStateLength);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
