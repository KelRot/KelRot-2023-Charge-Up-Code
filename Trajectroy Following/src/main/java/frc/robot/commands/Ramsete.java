package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Drive;

public class Ramsete extends CommandBase {
  private final Drive m_drive;
  private RamseteCommand m_ramsete;

  public Ramsete(Drive _drive) {
    m_drive = _drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        TrajectoryConstants.ksVolts,
        TrajectoryConstants.kvVoltSecondsPerMeter,
        TrajectoryConstants.kaVoltSecondsSquaredPerMeter
      ),
      TrajectoryConstants.kDriveKinematics,
      TrajectoryConstants.kMaxVoltage
    );

    TrajectoryConfig config = new TrajectoryConfig(
      TrajectoryConstants.kMaxSpeedMetersPerSecond,
      TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(TrajectoryConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint
    );

    Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      
      List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
      
      new Pose2d(3, 0, new Rotation2d(0)),
      
      config
    );

    m_ramsete = new RamseteCommand(
      autoTrajectory,
      m_drive::getPose,
      new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        TrajectoryConstants.ksVolts,
        TrajectoryConstants.kvVoltSecondsPerMeter,
        TrajectoryConstants.kaVoltSecondsSquaredPerMeter
      ),
      TrajectoryConstants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
      new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankDriveVolts,
      m_drive
    );
    
    m_drive.resetOdometry(autoTrajectory.getInitialPose());
  }

  @Override
  public void execute() {
    m_ramsete.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_ramsete.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_ramsete.isFinished();
  }
}
