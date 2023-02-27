package frc.robot.paths;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.TrajectoryConstants;

public class Path {
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

  public static class StraightLine {
    public static Pose2d kStart = new Pose2d(0, 0, new Rotation2d(0));
    
    public static List<Translation2d> kWayPoints = Arrays.asList(
      new Translation2d(1, 0)
    );
    
    public static Pose2d kEnd = new Pose2d(2, 0, new Rotation2d(0));
  }

  public static class S {
    public static Pose2d kStart = new Pose2d(0, 0, new Rotation2d(0));
    
    public static List<Translation2d> kWayPoints = Arrays.asList(
      new Translation2d(1, -1), 
      new Translation2d(2, 1)
    );
    
    public static Pose2d kEnd = new Pose2d(3, 0, new Rotation2d(0));
  }
}

