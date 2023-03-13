package frc.robot.paths;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Drive;

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

  public static Path straightLine = new Path(
    new Pose2d(0, 0, new Rotation2d(0)), 
    Arrays.asList(
      new Translation2d(1, 0),
      new Translation2d(2, 0)
    ),
    new Pose2d(3, 0, new Rotation2d(0)),
    false 
  );

  public static Path S = new Path(
    new Pose2d(0, 0, new Rotation2d(0)), 
    Arrays.asList(
      new Translation2d(1, 1),
      new Translation2d(2, -1)
    ),
    new Pose2d(3, 0, new Rotation2d(0)),
    false 
  );

  public static Path auto21 = new Path(
    new Pose2d(1.88, 4.38, new Rotation2d(180)), 
    Arrays.asList(
      new Translation2d(5.31, 4.59)
    ),
    new Pose2d(5.84, 3.07, Rotation2d.fromDegrees(0)),
    true
  );

  public static Command generateRamsete(Drive m_drive, Path m_path){

    Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
      m_path.kStart,

      m_path.kWayPoints,
      
      m_path.kEnd,

      m_path.kConfig
    );

    m_drive.m_field.getObject("traj").setTrajectory(autoTrajectory);

    RamseteController m_ramseteController = new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta);
    
    /* Debug */

    //m_ramseteController.setEnabled(false);

    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(TrajectoryConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(TrajectoryConstants.kPDriveVel, 0, 0);

    /* Debug */

    RamseteCommand m_ramsete = new RamseteCommand(
      autoTrajectory,
      m_drive::getPose,
      m_ramseteController,
      new SimpleMotorFeedforward(
        TrajectoryConstants.ksVolts,
        TrajectoryConstants.kvVoltSecondsPerMeter,
        TrajectoryConstants.kaVoltSecondsSquaredPerMeter
      ),
      TrajectoryConstants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      leftController,
      rightController,
      // RamseteCommand passes volts to the callback
      (leftVolts, rightVolts) -> {
        m_drive.tankDriveVolts(leftVolts, rightVolts);

        SmartDashboard.putNumber("Tank Drive Left Volt", leftVolts);
        SmartDashboard.putNumber("Tank Drive Right Volt", rightVolts);

        leftMeasurement.setNumber(m_drive.getWheelSpeeds().leftMetersPerSecond);
        leftReference.setNumber(leftController.getSetpoint());

        rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
        rightReference.setNumber(rightController.getSetpoint());

        m_drive.debug();
      },
      m_drive
    );
    
    m_drive.resetOdometry(autoTrajectory.getInitialPose());

    return m_ramsete.andThen(() -> m_drive.stopMotors());
  }

  public P(){}

}
