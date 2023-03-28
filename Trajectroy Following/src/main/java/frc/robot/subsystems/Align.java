package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinearPathConstants;

public class Align extends SubsystemBase{
  private final PhotonCamera m_camera = new PhotonCamera("gamepiece");

  private final double camHeight = 1.19;
  private final double targetHeight = 0.08; // 10cm 
  private final double camPitch = Units.degreesToRadians(-23); // hesaplanacak
  private final double intakeDistance = 1.10;
  private final double goalRange = Units.inchesToMeters(0); 

  private double m_prevAngle, m_prevDistance;

  private boolean m_hasTarget, m_isLatest;

  //drive motors

  public Align(){
    m_hasTarget = false;
  }

  public void changePipeline(boolean is_cube){
    if(is_cube)
      m_camera.setPipelineIndex(1);
    else
      m_camera.setPipelineIndex(0);
  }

  public double getAngle(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets() == true){
      m_hasTarget = true;
      return m_prevAngle = result.getBestTarget().getYaw();
    }
    return m_prevAngle;
  }

  public double getPitch(){
    var result = m_camera.getLatestResult();
    if(result.hasTargets() == true){
      m_hasTarget = true;
      double d = result.getBestTarget().getPitch();
      m_prevDistance = d;
    }
    return m_prevDistance;  
  }

  public boolean hasTarget(){
    var result = m_camera.getLatestResult();
    return result.hasTargets();
  }

  public boolean hasFoundAnyTarget(){
    return m_hasTarget;
  }

  public void periodic(){
    SmartDashboard.putNumber("Game Piece Angle", getAngle());
    SmartDashboard.putNumber("Game Piece Pitch", getPitch());
    if(m_camera.getPipelineIndex() == 1)
      SmartDashboard.putString("Game Piece", "Cube");
    else
      SmartDashboard.putString("Game Piece", "Cone");
    
  }
}
