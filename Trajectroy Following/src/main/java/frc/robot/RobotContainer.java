package frc.robot;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ChargingStation;
import frc.robot.commands.CyclindersFullClose;
import frc.robot.commands.CyclindersFullOpen;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LinearPathFollower.DriveTask;
import frc.robot.commands.OnePieceAutonomous;
import frc.robot.commands.OnePieceChargingMobility;
import frc.robot.commands.LinearPathFollower;
import frc.robot.paths.P;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;
import frc.robot.test_commands.DriveTaskFollower;
import frc.robot.test_commands.RotationTaskFollower;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Align m_vision = new Align();
  private final Pulley m_pulley = new Pulley();
  private final Pneumatics m_pneumatics = new Pneumatics();

  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_helicopter = new Joystick(1);

  private final DriveCommand driveCommand = new DriveCommand(m_drive, m_joystick);

  private final AlignCommand m_alignCommandCube = new AlignCommand(m_drive, m_vision, true);
  private final AlignCommand m_alignCommandCone = new AlignCommand(m_drive, m_vision, false);

  private final CyclindersFullOpen m_fullOpenCommand = new CyclindersFullOpen(m_pneumatics, m_pulley);
  private final CyclindersFullClose m_fullCloseCommand = new CyclindersFullClose(m_pneumatics, m_pulley);

  /* Auto commands */

  private final ChargingStation m_chargingStation = new ChargingStation(m_drive);

  private final AutoScore m_autoScore = new AutoScore(m_pneumatics, m_pulley);
  
  private final OnePieceChargingMobility m_onePieceC = new OnePieceChargingMobility(m_drive, m_pneumatics, m_pulley);

  private final OnePieceAutonomous m_onePieceAuto = new OnePieceAutonomous(m_drive, m_pneumatics, m_pulley);
  
  private final AprilTagVision m_aprilTagVision = new AprilTagVision(VisionConstants.kUpperCamera);
  
  private final LinearPathFollower m_linearPathFollower = new LinearPathFollower(m_drive, m_aprilTagVision);
  private final DriveTaskFollower m_driveTaskFollower = new DriveTaskFollower(m_drive); 
  private final RotationTaskFollower m_rotationTaskFollower = new RotationTaskFollower(m_drive);

  public RobotContainer() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    configureBindings();
    
    SmartDashboard.putData("Charging Station Balance", new ChargingStation(m_drive));
    SmartDashboard.putData("AutoScore", new AutoScore(m_pneumatics, m_pulley));

    SmartDashboard.putNumber("Drive Task X", 0.0);
    SmartDashboard.putNumber("Drive Task Y", 0.0);
    SmartDashboard.putNumber("Drive Task Distance", 0.0);
    SmartDashboard.putNumber("Rotation Task Degrees", 0.0);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(driveCommand);

    JoystickButton button[] = {
      new JoystickButton(m_joystick, 8), // toggle compressor
      new JoystickButton(m_helicopter, 5), // full open
      new JoystickButton(m_helicopter, 3), // full close
      new JoystickButton(m_helicopter, 1), // intake toggle
      new JoystickButton(m_joystick, 3), // slow sparks
      new JoystickButton(m_joystick, 1), // middle sparks
      new JoystickButton(m_joystick, 2), // fast sparks   
      new JoystickButton(m_joystick, 4), // brake mode toggle
      new JoystickButton(m_joystick, 5), // odometry button
      new JoystickButton(m_joystick, 6), // charging run
      new JoystickButton(m_joystick, 7) // charging stop
    };

    POVButton pov[] = {
      new POVButton(m_joystick, 0), // pulley reset
      new POVButton(m_joystick, 180), // pulley open
      new POVButton(m_joystick, 90), // pulley close
      new POVButton(m_helicopter, 0),
      new POVButton(m_helicopter, 90),
      new POVButton(m_helicopter, 180),
      new POVButton(m_helicopter, 270),
    };

    JoystickButton byHand[] = {
      new JoystickButton(m_helicopter, 8), //intake toogle
      new JoystickButton(m_helicopter, 10), //telescope toogle
      new JoystickButton(m_helicopter, 12), //arm toogle
      new JoystickButton(m_helicopter, 4), //pulley close
      new JoystickButton(m_helicopter, 6), //pulley open
      new JoystickButton(m_helicopter, 7),
      new JoystickButton(m_helicopter, 9)
    };

    byHand[0].whileTrue(new InstantCommand(() -> m_pneumatics.getIntakeSolenoid().toggle()));
    byHand[1].whileTrue(new InstantCommand(() -> m_pneumatics.getTelescopeSolenoid().toggle())); 
    byHand[2].whileTrue(new InstantCommand(() -> m_pneumatics.getArmSolenoid().toggle()));

    byHand[3].whileTrue(new InstantCommand(() -> m_pulley.openPulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));
    byHand[4].whileTrue(new InstantCommand(() -> m_pulley.closePulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));

    pov[3].onTrue(m_linearPathFollower);
    pov[5].onTrue(m_driveTaskFollower);
    byHand[5].whileTrue(m_rotationTaskFollower);
    byHand[6].whileTrue(new InstantCommand (() -> m_rotationTaskFollower.cancel()));

    button[0].whileTrue(new InstantCommand(() -> m_pneumatics.toggleCompressor()));
    button[1].onTrue(m_fullOpenCommand);
    button[2].onTrue(m_fullCloseCommand);
    button[3].whileTrue(new InstantCommand(() -> m_pneumatics.getIntakeSolenoid().toggle()));

    /* speed */
    button[4].whileTrue(new InstantCommand(() -> m_drive.setSlowMode()));
    button[5].whileTrue(new InstantCommand(() -> m_drive.setNormalMode()));
    button[6].whileTrue(new InstantCommand(() -> m_drive.setFastMode()));

    /* brake */
    button[7].whileTrue(new InstantCommand(() -> m_drive.changeIdleMode()));
    
    /* reset odometry */
    button[8].whileTrue(new InstantCommand(() -> m_drive.resetOdometry()));

    
    button[9].onTrue(m_chargingStation);
    button[10].onTrue(new InstantCommand(() -> m_chargingStation.cancel()));
    pov[0].onTrue(new InstantCommand(() -> m_pulley.reset()));

    pov[1].whileTrue(new InstantCommand(() -> m_pulley.openPulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));
    pov[2].whileTrue(new InstantCommand(() -> m_pulley.closePulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));
    
  }

  public Command getAutonomousCommand() {
    return P.generateRamsete(m_drive, P.straightLine);
    /*Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
      P.S.kStart,

      P.S.kWayPoints,
      
      P.S.kEnd,

      P.S.kConfig
    );

    m_drive.m_field.getObject("traj").setTrajectory(autoTrajectory);


    RamseteController m_ramseteController = new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta);
    
    

    //m_ramseteController.setEnabled(false);

    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(TrajectoryConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(TrajectoryConstants.kPDriveVel, 0, 0);

   

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

    return m_ramsete.andThen(() -> m_drive.stopMotors());*/
  }

  public void testPeriodic() {
    var volts = SmartDashboard.getNumber("Test Volts", 0);
    m_drive.tankDriveVolts(volts, volts);
  }

}