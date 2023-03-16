package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Align extends SubsystemBase{

  final double camHeight = Units.inchesToMeters(0);
  final double targetHight = Units.inchesToMeters(0);
  final double camPitch = Units.degreesToRadians(0);
  final double goalRange = Units.inchesToMeters(0);

  private final PhotonCamera m_camera;
  //drive motors

  public Align(){
    m_camera = new PhotonCamera("gamepiece");
  }

  public double getAngle(){
    var result = m_camera.getLatestResult();
    return result.getBestTarget().getYaw();
  }
    
}
