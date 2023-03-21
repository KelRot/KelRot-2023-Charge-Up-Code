package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.simulation.REVPHDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private AprilTagFieldLayout fieldLayout;
    private final Drive m_drive;

    public AprilTagVision(Drive drive) {
        photonCamera = new PhotonCamera("apriltag");
        m_drive = drive;
        try {
            fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
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

    public Pose3d getTagRelativePose() {
        if(fieldLayout.getTagPose(targetID).isPresent()) {
            Pose3d aprilTagPose = fieldLayout.getTagPose(targetID).get();
            return PhotonUtils.estimateFieldToRobotAprilTag(bestCameraToTarget, aprilTagPose, VisionConstants.robotToCam);
        }
        else {
            return new Pose3d();
        }
    }

    public void debug() {
        SmartDashboard.putBoolean("Upper Camera Target", hasTargets);
        if(hasTargets){
            SmartDashboard.putNumber("Upper Camera Yaw", yaw);
            SmartDashboard.putNumber("Upper Camera Area", area);
            SmartDashboard.putNumber("Upper Camera ID", targetID);
            try {
                AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                Pose3d m_aprilTagPose = fieldLayout.getTagPose(targetID).get();
                SmartDashboard.putString("Upper Camera April X-Y-Z",    
            (m_aprilTagPose.getX() + " " +
            m_aprilTagPose.getY() + " " +
            m_aprilTagPose.getZ() ));
            } catch (IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            }
            SmartDashboard.putNumber("Upper Camera Yaw", yaw);
            SmartDashboard.putNumber("Upper Camera Ambiguity", poseAmbiguity);
            SmartDashboard.putString("Upper Camera X-Y-Z",    
            (bestCameraToTarget.getX() + " " +
            bestCameraToTarget.getY() + " " +
            bestCameraToTarget.getZ() ));
            Pose3d robotPose = getTagRelativePose();
            SmartDashboard.putString("Robot Pose", "X: " + robotPose.getX() + " Y: " + robotPose.getY());
        }
    }

    @Override
    public void periodic() {   
        getLatestResult();
        Pose3d pose = getTagRelativePose();
        m_drive.setRobotPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(pose.getRotation().getAngle())));
        debug();
    }
    
}
