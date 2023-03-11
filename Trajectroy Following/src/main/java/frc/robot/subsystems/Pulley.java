package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Pulley extends SubsystemBase {
    private final PWMVictorSPX m_pgController = new PWMVictorSPX(0);

    public Pulley() {

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
}
