package frc.robot.commands;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class CubeSecondNode extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Joystick m_js;
  private final Drive m_drive;
  private final Timer m_timer = new Timer();
  private boolean m_finished, m_closing = false;

  public CubeSecondNode(Pneumatics pneumatics, Pulley pulley, Joystick js, Drive drive) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;
    m_js = js;
    addRequirements(m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_finished = false;
    m_closing = false;

    m_pulley.set(PulleyConstants.kArmOpenStateLength);

    SmartDashboard.putBoolean("Cube Second", true);
  }

  @Override
  public void execute() {
    if(m_closing == false){
      if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength - 200.0){
        m_pneumatics.getArmSolenoid().open();
        m_timer.start();
      }
      if(m_timer.get() >= 0.9) {
        m_pneumatics.getIntakeSolenoid().open();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
        m_closing = true;
      }
    }else{
      if(m_pulley.getDistance() <= 1100.0 && m_closing == true){
        m_pneumatics.getArmSolenoid().close();
        m_drive.drive(m_js);
        m_finished = true;
      }
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Cube Second", false);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
