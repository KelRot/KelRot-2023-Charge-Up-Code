package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class AprilTagVision extends SubsystemBase {
    

    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    
    public boolean hasTargets;
    private double yaw;
    private double pitch;
    private double area;
    private int targetID;
    private double poseAmbiguity;
    private Transform3d bestCameraToTarget;

    public AprilTagVision(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public double getYaw() { return yaw; }
    public double getPitch() { return pitch; }
    public double getArea() { return area; }
    public int getId() { return targetID; }
    public double getPoseAmbiguity() { return poseAmbiguity; }
    public Transform3d getPose() { return bestCameraToTarget; }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void getLatestResult() {
        var result = photonCamera.getLatestResult();
        hasTargets = result.hasTargets();
        if(hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget(); // List<PhotonTrackedTarget> targets = result.getTargets();
        
            yaw = target.getYaw();
            pitch = target.getPitch();
            area = target.getArea();
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        else {
            yaw = 0;
            pitch = 0;
            area = 0;
            targetID = 0;
            poseAmbiguity = 0;
            bestCameraToTarget = null;
        }
    }

    public void debug() {
        SmartDashboard.putBoolean("Upper Camera Target", hasTargets);
        SmartDashboard.putNumber("Upper Camera Yaw", yaw);
        SmartDashboard.putNumber("Upper Camera Pitch", pitch);
        SmartDashboard.putNumber("Upper Camera Area", area);
        SmartDashboard.putNumber("Upper Camera ID", targetID);
        SmartDashboard.putNumber("Upper Camera Ambiguity", poseAmbiguity);
        SmartDashboard.putNumberArray("Upper Camera X-Y-Z", new double[] {   
        bestCameraToTarget.getX(),
        bestCameraToTarget.getY(),
        bestCameraToTarget.getZ() });
    }

    @Override
    public void periodic() {   
        getLatestResult();

    }
    
}
