// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  private final Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final Solenoid dArmSolenoid, dTelescopSolenoid, dIntakeSolenoid;

  public final class Solenoid{
    private final DoubleSolenoid m_sol;
    private boolean m_state;
    private final String m_name;
    
    public Solenoid(int[] pin, String name){
      m_sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pin[0], pin[1]);
      m_state = false;
      m_name = name;
      setSolenoid("reverse", m_sol);
    }

    public void toggle(){
      m_sol.toggle();
      m_state = !m_state;
    }
    
    public void open(){
      m_sol.set(kForward);
      m_state = true;
    }
    
    public void close(){
      m_sol.set(kReverse);
      m_state = false;
    }

    public boolean getState(){
      return m_state;
    }

    private void debug(){
      SmartDashboard.putBoolean(m_name, m_state);
    }
  };

  public Pneumatics() { 
    m_compressor.disable();

    dArmSolenoid = new Solenoid(PneumaticsConstants.kArmPins, "Arm");
    dTelescopSolenoid = new Solenoid(PneumaticsConstants.kTelescopePins, "Telescope");
    dIntakeSolenoid = new Solenoid(PneumaticsConstants.kIntakePins, "Intake");
  }
  
  public void setSolenoid(String state, DoubleSolenoid dSolenoid) {
    if(state == "forward") {
      dSolenoid.set(kForward);
    } else if (state == "reverse") {
      dSolenoid.set(kReverse);
    } else {
      dSolenoid.set(kOff);
    }
  }

  public Solenoid getArmSolenoid() {
    return dArmSolenoid;
  }

  public Solenoid getTelescopeSolenoid() {
    return dTelescopSolenoid;
  }

  public Solenoid getIntakeSolenoid() {
    return dIntakeSolenoid;
  }

  public double getCurrent() {
    return m_compressor.getCurrent();
  }

  public boolean getPressureSwitch() {
    return m_compressor.getPressureSwitchValue();
  }

  public void toggleCompressor(){
    if(m_compressor.isEnabled())
      m_compressor.disable();
    else
      m_compressor.enableDigital();
  }

  @Override
  public void periodic() {
    dIntakeSolenoid.debug();
    dTelescopSolenoid.debug();
    dArmSolenoid.debug();
  }
}
