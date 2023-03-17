package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Align extends SubsystemBase{
  private final PhotonCamera m_camera = new PhotonCamera("gamepiece");

  private final double camHeight = Units.inchesToMeters(0);
  private final double targetHeight = Units.inchesToMeters(0);
  private final double camPitch = Units.degreesToRadians(0);
  private final double goalRange = Units.inchesToMeters(0);

  private double m_prevAngle, m_prevDistance;

  //drive motors

  public Align(){}

  public void changePipeline(boolean is_cube){
    if(is_cube)
      m_camera.setPipelineIndex(0);
    else
      m_camera.setPipelineIndex(1);
  }

  public double getAngle(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets()){
      return m_prevAngle = result.getBestTarget().getYaw();
    }
    return m_prevAngle;
  }

  public double getDistance(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets()){

      double d = PhotonUtils.calculateDistanceToTargetMeters(
        camHeight,
        targetHeight,
        camPitch,
        Units.degreesToRadians(result.getBestTarget().getPitch())
      );

      return m_prevDistance = d;
    }
    return m_prevDistance;  
  }
}
