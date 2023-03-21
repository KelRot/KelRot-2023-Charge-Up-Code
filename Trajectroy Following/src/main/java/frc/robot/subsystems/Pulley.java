package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PulleyConstants;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pulley extends SubsystemBase {
  private final PWMVictorSPX m_pgController;
  private final Encoder m_encoder;
  private double m_speed, m_setPoint;
  private boolean m_isMoving;

  // encoder is 0 at the full closed state
  // encoder is max at the full open state

  public Pulley() {
    m_pgController = new PWMVictorSPX(0);
    m_encoder = new Encoder(PulleyConstants.kA, PulleyConstants.kB);
    m_encoder.setDistancePerPulse(1);
    m_encoder.reset();
    m_speed = 0.0;
    m_setPoint = 0.0;
    m_isMoving = false;
  }

  public void openPulley(){
    m_speed = -12.0;
  }

  public void fixedPulley(){
    m_speed = 0.1;
  }

  public void stopPulley() {
    m_speed = 0.0;
  }

  public void slowClosePulley(){
    m_speed = 0.25;
  }

  public void closePulley(){
    m_speed = 12.0;
  }

  public void set(double set_point){
    double d = getDistance() - set_point;
    m_setPoint = set_point;
    if(Math.abs(d) >= PulleyConstants.kTolerance){
      m_isMoving = true;
      if(d < 0){
        openPulley();
      }else{
        closePulley();
      }
    }else{
      m_isMoving = false;
      m_setPoint = 0.0;
      m_speed = 0.0;
    }
  }

  public double getDistance(){
    return m_encoder.getDistance();
  }

  public void runPulley(){
    if(m_isMoving){
      set(m_setPoint);
    }
    m_pgController.set(m_speed);
  }

  public void reset(){
    m_encoder.reset();
    m_setPoint = 0.0;
    m_speed = 0.0;
    m_isMoving = false;
  }

  public void debug(){
    SmartDashboard.putNumber("Pulley", m_encoder.getDistance());
  }

  @Override
  public void periodic(){
    runPulley();
    debug();
  }
}
