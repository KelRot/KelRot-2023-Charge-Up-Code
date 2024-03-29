package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargingConstants;
import frc.robot.Constants.PulleyConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

public class OnePieceChargingMobility extends CommandBase {
  private final Drive m_drive;
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final PIDController m_pid;
  private final Timer m_timer = new Timer(), m_autoTimer = new Timer();
  private boolean m_finished, m_closing, m_isReached, m_isPassedCS, m_gettingOff, m_climbing;

  public OnePieceChargingMobility(Drive drive, Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;

    m_pid = new PIDController(0.35, ChargingConstants.kI, 0.054);
    m_pid.setTolerance(6, 5);

    SmartDashboard.putString("One Piece Auto", "Not begun");

    addRequirements(m_drive, m_pneumatics, m_pulley);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("One Piece Auto", "Initialized");

    m_timer.stop();
    m_timer.reset();
    m_pulley.reset();
    m_pulley.set(PulleyConstants.kFullOpenStateLength);
    m_drive.setFastMode();
    m_drive.setIdleModeBrake(false);
    m_drive.setGyroAxis(IMUAxis.kY);
    m_drive.resetGyro();
    m_pid.reset();

    m_finished = m_closing = m_isReached = m_isPassedCS = m_climbing = m_gettingOff = false;

    m_autoTimer.reset();
    m_autoTimer.start();
    SmartDashboard.putNumber("Auto Time", m_autoTimer.get());
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
      m_drive.tankDriveVolts(0, 0);
    }else{
      if(m_pulley.getDistance() <= 1200){//OKOKKOKOKOKOKOKO
        m_pneumatics.getArmSolenoid().close();
      }
      if(m_isPassedCS == false){
        m_drive.setIdleModeBrake(true);
        double chargingVolts = 6.0;
        m_drive.tankDriveVolts(-chargingVolts, -chargingVolts);
        if(Math.abs(m_drive.getAngle()) > 10){
          m_climbing = true;
          SmartDashboard.putString("One Piece Auto", "Climbing");
        }
        if(m_climbing == true){
          if(Math.abs(m_drive.getAngle()) <= 1){
            m_isReached = true;
            SmartDashboard.putString("One Piece Auto", "On charging");
          }
        }
        if(m_isReached == true){
          if(Math.abs(m_drive.getAngle()) > 10){
            m_gettingOff = true;
            SmartDashboard.putString("One Piece Auto", "Getting off");
          }
        }
        if(m_gettingOff == true){
          if(Math.abs(m_drive.getAngle()) <= 1){
            m_isPassedCS = true;
            m_isReached = false;
            SmartDashboard.putString("One Piece Auto", "Passed CS");
          }
        }
      }else{
        if(m_isReached == false) {
          double chargingVolts = 7.8;
          m_drive.tankDriveVolts(chargingVolts, chargingVolts);
          if(Math.abs(m_drive.getAngle()) > 10)
            m_isReached = true;
        }
        else {
          double volts = -m_pid.calculate(m_drive.getAngle(), 0);
          m_drive.tankDriveVolts(volts, volts);
          if(m_pid.atSetpoint()){
            m_finished = true;
          }
        }
      }
    }
    SmartDashboard.putBoolean("Is reached", m_isReached);
    SmartDashboard.putNumber("Auto Time", m_autoTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Charging State", "Finished");
    m_drive.stopMotors();
    m_drive.setGyroAxis(IMUAxis.kZ);
    m_drive.setIdleModeBrake(true);
    m_pulley.reset();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
