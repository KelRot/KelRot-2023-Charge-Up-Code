package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PulleyConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Pulley extends SubsystemBase {
  private final PWMVictorSPX m_pgController = new PWMVictorSPX(0);
  private final Encoder m_encoder = new Encoder(PulleyConstants.kA, PulleyConstants.kB);

  // encoder is 0 at the full closed state
  // encoder is max at the full open state

  public Pulley() {
    m_encoder.reset();
  }

  public void setMotorVolts(double volts) {
    m_pgController.setVoltage(volts);;
  }

  public void openPulley(){
    m_pgController.setVoltage(12.0);
  }

  public void stopPulley(){
    m_pgController.setVoltage(0.0);
  }

  public void closePulley(){
    m_pgController.setVoltage(-12.0);
  }

  public void set(double set_point){
    double d = m_encoder.getDistance() - set_point;
    if(Math.abs(d) >= PulleyConstants.kTolerance){
      if(d > 0){
        openPulley();
      }else{
        closePulley();
      }
    }
  }
}
