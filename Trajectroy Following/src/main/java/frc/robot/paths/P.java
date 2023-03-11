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
  private static final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
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
    .addConstraint(voltageConstraint);


  public static class Path{
    public Pose2d kStart;
    public List<Translation2d> kWayPoints;
    public Pose2d kEnd;
    public TrajectoryConfig kConfig;

    public Path(Pose2d start, List<Translation2d> wayPoints, Pose2d end, boolean startReversed){
      kStart = start;
      kWayPoints = wayPoints;
      kEnd = end;
      kConfig = config;
      kConfig.setReversed(startReversed);
    }
  }

  public static Path straightLine, S, auto21;

  public P(){
    straightLine = new Path(
      new Pose2d(0, 0, new Rotation2d(0)), 
      Arrays.asList(
        new Translation2d(1, 0),
        new Translation2d(2, 0)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      false 
    );

    S = new Path(
      new Pose2d(0, 0, new Rotation2d(0)), 
      Arrays.asList(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      false 
    );

    auto21 = new Path(
      new Pose2d(0, 0, new Rotation2d(0)), 
      Arrays.asList(
        new Translation2d(5, 0)
      ),
      new Pose2d(5, 1.6, new Rotation2d(0)),
      true
    );
  }

}
