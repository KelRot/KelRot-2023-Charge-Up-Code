package frc.robot;

import frc.robot.commands.AutoScore;
import frc.robot.commands.ChargingStation;
import frc.robot.commands.CyclindersFullClose;
import frc.robot.commands.CyclindersFullOpen;
import frc.robot.commands.DriveCommand;
import frc.robot.paths.P;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.net.PortForwarder;

public class RobotContainer {
  private final Drive m_drive = new Drive();

  private final Pneumatics m_pneumatics = new Pneumatics();
  private final Pulley m_pulley = new Pulley();

  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_helicopter = new Joystick(1);

  private final DriveCommand driveCommand = new DriveCommand(m_drive, m_joystick);

  private final CyclindersFullOpen m_fullOpenCommand = new CyclindersFullOpen(m_pneumatics);
  private final CyclindersFullClose m_fullCloseCommand = new CyclindersFullClose(m_pneumatics);

  private final Command m_pathFollowerLine = P.generateRamsete(m_drive, P.straightLine);
  private final Command m_pathFollowerS = P.generateRamsete(m_drive, P.S);

  private final Command m_pathFollower21Path = P.generateRamsete(m_drive, P.auto21);
  private final ChargingStation m_chargingStation = new ChargingStation(m_drive);
  private final AutoScore m_autoScore = new AutoScore(m_pneumatics, m_pulley);

  /*private final SequentialCommandGroup m_21auto = new SequentialCommandGroup(
    m_autoScore,
    m_pathFollower21Path,
    m_chargingStation
  );*/

  public RobotContainer() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    configureBindings();
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(driveCommand);

    JoystickButton button[] = {
      new JoystickButton(m_joystick, 5), // toggle compressor
      new JoystickButton(m_helicopter, 5), // full open
      new JoystickButton(m_helicopter, 3), // full close
      new JoystickButton(m_helicopter, 1), // intake toggle
      new JoystickButton(m_joystick, 1), // slow sparks
      new JoystickButton(m_joystick, 2), // middle sparks
      new JoystickButton(m_joystick, 3), // fast sparks   
      new JoystickButton(m_joystick, 4), // brake mode toggle
      new JoystickButton(m_joystick, 7), // odometry button
      new JoystickButton(m_joystick, 8), // charging run
      new JoystickButton(m_joystick, 9) // charging stop
    };

    JoystickButton byHand[] = {
      new JoystickButton(m_helicopter, 8), //intake toogle
      new JoystickButton(m_helicopter, 10), //telescope toogle
      new JoystickButton(m_helicopter, 12), //arm toogle
      new JoystickButton(m_helicopter, 6), //pulley close
      new JoystickButton(m_helicopter, 4) //pulley open
    };

    byHand[0].whileTrue(new InstantCommand(() -> m_pneumatics.getIntakeSolenoid().toggle()));
    byHand[1].whileTrue(new InstantCommand(() -> m_pneumatics.getTelescopeSolenoid().toggle()));
    byHand[2].whileTrue(new InstantCommand(() -> m_pneumatics.getArmSolenoid().toggle()));

    byHand[3].whileTrue(new InstantCommand(() -> m_pulley.openPulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));
    byHand[4].whileTrue(new InstantCommand(() -> m_pulley.closePulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));

    /* */
    button[0].whileTrue(new InstantCommand(() -> m_pneumatics.toggleCompressor()));
    button[1].onTrue(m_fullOpenCommand);
    button[2].onTrue(m_fullCloseCommand);
    button[3].whileTrue(new InstantCommand(() -> m_pneumatics.getIntakeSolenoid().toggle()));

    /* speed */
    button[4].whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(4.0)));
    button[5].whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(8.0)));
    button[6].whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(12.0)));

    /* brake */
    button[7].whileTrue(new InstantCommand(() -> m_drive.changeIdleMode()));
    
    /* reset odometry */
    button[8].whileTrue(new InstantCommand(() -> m_drive.resetOdometry()));

    
    button[9].debounce(0.5).onTrue(m_chargingStation);
    button[10].debounce(0.5).onTrue(new InstantCommand(() -> m_chargingStation.cancel()));
    
  }

  public Command getAutonomousCommand() {
    return m_pathFollower21Path;
    /*Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
      P.auto21.kStart,

      P.auto21.kWayPoints,
      
      P.auto21.kEnd,

      P.auto21.kConfig
    );

    m_drive.m_field.getObject("traj").setTrajectory(autoTrajectory);


    RamseteController m_ramseteController = new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta);
    
    

    m_ramseteController.setEnabled(false);

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

    return m_ramsete.andThen(() -> m_drive.stopMotors()); */
  }
}