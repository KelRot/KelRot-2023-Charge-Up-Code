package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class ConeSecondNode extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Drive m_drive;
  private final Pulley m_pulley;
  private final Timer m_timer = new Timer();
  private boolean m_finished = false;

  public ConeSecondNode(Pneumatics pneumatics, Pulley pulley, Drive drive) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;
    addRequirements(m_pneumatics, m_pulley, m_drive);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_finished = false;

    m_pulley.set(PulleyConstants.kConeTwoState);

    SmartDashboard.putBoolean("Cone Second", true);
  }

  @Override
  public void execute() {
    m_drive.curvatureDrive(0.2, 0);
    if(m_pulley.getDistance() >= PulleyConstants.kConeTwoState){
      m_timer.start();
      m_pulley.openPulley();
      m_pneumatics.getArmSolenoid().open();
    }
    if(m_timer.get() >= 0.46){
      m_pneumatics.getIntakeSolenoid().open();
      m_pulley.set(PulleyConstants.kFullCloseStateLength);
    }
    if(m_timer.get() >= 1.1){
      m_pneumatics.getArmSolenoid().close();
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
