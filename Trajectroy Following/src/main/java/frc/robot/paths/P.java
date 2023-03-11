package frc.robot.paths;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.TrajectoryConstants;

public class P {
  private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
      TrajectoryConstants.ksVolts,
      TrajectoryConstants.kvVoltSecondsPerMeter,
      TrajectoryConstants.kaVoltSecondsSquaredPerMeter
    ),
    TrajectoryConstants.kDriveKinematics,
    TrajectoryConstants.kMaxVoltage
  );

  public static final TrajectoryConfig config = new TrajectoryConfig(
    TrajectoryConstants.kMaxSpeedMetersPerSecond,
    TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(TrajectoryConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

  // 21 Point path

  public Trajectory traj21 = new Trajectory();
  String trajectoryJSON = "pathplanner/generatedJSON/path21.wpilib.json";
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);

  public P(){
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      traj21 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  /////////////////

  public static class StraightLine {
    public static Pose2d kStart = new Pose2d(0, 0, new Rotation2d(0));
    
    public static List<Translation2d> kWayPoints = Arrays.asList(
      new Translation2d(1, 0),
      new Translation2d(2, 0.5)
    );
    
    public static Pose2d kEnd = new Pose2d(3, 1, new Rotation2d(0));
  }

  public static class S {
    public static Pose2d kStart = new Pose2d(0, 0, new Rotation2d(0));
    
    public static List<Translation2d> kWayPoints = Arrays.asList(
      new Translation2d(1, 0.5), 
      new Translation2d(2, -0.5)
    );
    
    public static Pose2d kEnd = new Pose2d(3, 0, new Rotation2d(0));
  }

  public static class Otonom21 {
    public static Pose2d kStart = new Pose2d(0, 0, new Rotation2d(0));
    
    public static List<Translation2d> kWayPoints = Arrays.asList(
      new Translation2d(5, 1)
    );
    
    public static Pose2d kEnd = new Pose2d(5, -1.6, new Rotation2d(0));
  }
}
