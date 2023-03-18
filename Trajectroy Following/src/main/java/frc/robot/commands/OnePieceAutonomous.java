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

public class OnePieceAutonomous extends CommandBase {
  private final Drive m_drive;
  private final Pneumatics m_pneumatics;
  private final Pulley m_pulley;
  private final PIDController m_pid;
  private final Timer m_timer = new Timer(), m_autoTimer = new Timer();
  private boolean m_finished, m_closing, m_isReached;

  public OnePieceAutonomous(Drive drive, Pneumatics pneumatics, Pulley pulley) {
    m_pneumatics = pneumatics;
    m_pulley = pulley;
    m_drive = drive;

    m_pid = new PIDController(ChargingConstants.kP, ChargingConstants.kI, ChargingConstants.kD);
    m_pid.setTolerance(6, 5);

    m_finished = m_closing = m_isReached = false;

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
    m_drive.setNormalMode();
    m_drive.setGyroAxis(IMUAxis.kY);
    m_drive.resetGyro();
    m_pid.reset();

    m_closing = false;

    m_autoTimer.reset();
    m_autoTimer.start();
    SmartDashboard.putNumber("Auto Time", m_autoTimer.get());
  }

  @Override
  public void execute() {
    if(!m_closing){

      if(m_pulley.getDistance() >= PulleyConstants.kArmOpenStateLength){
        m_pneumatics.getArmSolenoid().open();
      }
      if(m_pulley.getDistance() >= PulleyConstants.kFullOpenStateLength - PulleyConstants.kMomentumTolerance){
        m_pneumatics.getTelescopeSolenoid().open();
        m_pneumatics.getIntakeSolenoid().open();
        m_timer.start();
      }
      if(m_timer.get() >= 0.2){
        m_pneumatics.getTelescopeSolenoid().close();
        m_pulley.set(PulleyConstants.kFullCloseStateLength);
        m_closing = true;
        SmartDashboard.putString("One Piece Auto", "Piece Scored");
      }
    }else{
      
      if(m_pulley.getDistance() <= PulleyConstants.kOnGroundStateLength){
        m_pneumatics.getArmSolenoid().close();
      }
      if(m_isReached == false) {
        m_drive.tankDriveVolts(ChargingConstants.kRequiredVoltage, ChargingConstants.kRequiredVoltage);
        if(Math.abs(m_drive.getAngle()) > ChargingConstants.kRequiredAngle)
          m_isReached = true;
      }
      else {
        double volts = m_pid.calculate(m_drive.getAngle(), 0);
        m_drive.tankDriveVolts(volts, volts);
        if(m_pid.atSetpoint()){
          m_finished = true;
        }
      }
    }
    
    SmartDashboard.putNumber("Auto Time", m_autoTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Charging State", "Finished");
    m_drive.setIdleModeBrake(true);
    m_drive.setGyroAxis(IMUAxis.kZ);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
