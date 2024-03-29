package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class CubeThirdNode extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Timer m_timer = new Timer();
  private final Drive m_drive;
  private final Joystick m_js;
  private boolean m_finished = false, m_closing;

  public CubeThirdNode(Pneumatics pneumatics, Pulley pulley, Drive drive, Joystick js) {
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

    m_pulley.set(PulleyConstants.kFullOpenStateLength);

    SmartDashboard.putBoolean("Cube Third", true);
  }

  @Override
  public void execute() {
    if(!m_closing){
      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - 1500.0){ //OKOKOKOKOKOKOK
        m_pneumatics.getArmSolenoid().open();
      }
      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - 500.0){
        m_pneumatics.getTelescopeSolenoid().open();
        m_timer.start();
      }
      if(m_timer.get() >= 0.3){
        m_pneumatics.getIntakeSolenoid().open();
      }
      if(m_timer.get() >= 0.6){
        m_pneumatics.getTelescopeSolenoid().close();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
        m_closing = true;
      }
    }else{
      if(m_pulley.getDistance() <= 1200){//OKOKKOKOKOKOKOKO
        m_pneumatics.getArmSolenoid().close();
        m_finished = true;
      }
      m_drive.drive(m_js);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Cube Third", false);
    m_drive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
