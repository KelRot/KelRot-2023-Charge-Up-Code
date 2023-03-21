package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Align extends SubsystemBase{
  private final PhotonCamera m_camera = new PhotonCamera("gamepiece");

  private final double camHeight = 0.0;
  private final double targetHeight = 0.10; // 10cm 
  private final double camPitch = Units.degreesToRadians(0); // hesaplanacak
  private final double goalRange = Units.inchesToMeters(0); 

  private double m_prevAngle, m_prevDistance;

  private boolean m_hasTarget;

  //drive motors

  public Align(){
    m_hasTarget = false;
  }

  public void changePipeline(boolean is_cube){
    if(is_cube)
      m_camera.setPipelineIndex(0);
    else
      m_camera.setPipelineIndex(1);
  }

  public double getAngle(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets()){
      m_hasTarget = true;
      return m_prevAngle = result.getBestTarget().getYaw();
    }
    return m_prevAngle;
  }

  public double getDistance(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets()){
      m_hasTarget = true;
      double d = PhotonUtils.calculateDistanceToTargetMeters(
        camHeight,
        targetHeight,
        camPitch,
        Units.degreesToRadians(result.getBestTarget().getPitch())
      );
      m_prevDistance = d;
    }
    return m_prevDistance;  
  }

  public boolean hasFoundAnyTarget(){
    return m_hasTarget;
  }

  public void periodic(){
    SmartDashboard.putNumber("Game Piece Distance", m_prevDistance);
    SmartDashboard.putNumber("Game Piece Angle", m_prevAngle);
    if(m_camera.getPipelineIndex() == 0)
      SmartDashboard.putString("Game Piece", "Cube");
    else
      SmartDashboard.putString("Game Piece", "Cone");
    
  }
}
