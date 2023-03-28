package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class ConeThirdNode extends CommandBase {
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final Drive m_drive;
  private final Joystick m_js;
  private final Timer m_timer = new Timer();
  private boolean m_finished, m_closing = false;

  public ConeThirdNode(Pneumatics pneumatics, Pulley pulley, Drive drive, Joystick js) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;
    m_js = js;
    addRequirements(m_pneumatics, m_pulley, m_drive);
  }

  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();

    m_finished = false;
    m_closing = false;

    

    m_pulley.set(PulleyConstants.kFullOpenStateLength);

    SmartDashboard.putBoolean("Cone Third", true);

  }

  @Override
  public void execute() {
    /*if(m_closing == false) {

      if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength){
        m_pneumatics.getArmSolenoid().open(); 
      }

      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - 300.0){
        m_pneumatics.getTelescopeSolenoid().open();
        m_timer.start();
        m_closing = true;
      }

    } else {
    
      if(m_timer.get() >= 0.7){
        m_pneumatics.getIntakeSolenoid().open();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
      }

      if(m_timer.get() >= 1.1){
        m_pneumatics.getArmSolenoid().close();
      }
      else if(m_timer.get() >= 0.8) {
        m_pneumatics.getArmSolenoid().open();
      }
      else if(m_timer.get() >= 0.5){
        m_pneumatics.getArmSolenoid().close();
      }

      if(m_timer.get() >= 0.9){
        m_pneumatics.getTelescopeSolenoid().close();
      }
    */
    m_drive.drive(m_js);
    if(m_closing == false) {

      if(m_pulley.getDistance() >=  PulleyConstants.kFullOpenStateLength - 2200.0){
        m_pneumatics.getArmSolenoid().open(); 
      }

      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - 300.0){
        m_pneumatics.getTelescopeSolenoid().open();
        m_timer.start();
        m_closing = true;
      }

    } else {
      

      if(m_timer.get() >= 1.5){
        m_pneumatics.getIntakeSolenoid().open();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
      }

      if(m_timer.get() >= 1.8){
        m_pneumatics.getTelescopeSolenoid().close();
      }
      
      if(m_timer.get() >= 5.0){
        m_pneumatics.getArmSolenoid().close();
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
