package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class CyclindersFullClose extends CommandBase {
  private final Pneumatics m_pneumatics;

  public CyclindersFullClose(Pneumatics pneumatics) {
    m_pneumatics = pneumatics;
    addRequirements(m_pneumatics);
  }

  @Override
  public void initialize() {
    m_pneumatics.getTelescopeSolenoid().close();
    m_pneumatics.getArmSolenoid().close();
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
