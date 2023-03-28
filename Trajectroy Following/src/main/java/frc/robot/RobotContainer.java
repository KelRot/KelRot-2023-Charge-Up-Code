package frc.robot;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ChargingBrake;
import frc.robot.commands.ChargingStation;
import frc.robot.commands.ConeSecondNode;
import frc.robot.commands.ConeThirdNode;
import frc.robot.commands.CubeSecondNode;
import frc.robot.commands.CubeThirdNode;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LinearPathFollower.DriveTask;
import frc.robot.commands.OnePieceAutonomous;
import frc.robot.commands.OnePieceChargingMobility;
import frc.robot.commands.OnlyCube;
import frc.robot.commands.StraightDrive;
import frc.robot.commands.Turn180;
import frc.robot.commands.TwoPieceAutoA;
import frc.robot.commands.LinearPathFollower;
import frc.robot.commands.OnePiece18;
import frc.robot.paths.P;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;
import frc.robot.test_commands.AutoScore;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;

public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Align m_vision = new Align();
  private final Pulley m_pulley = new Pulley();
  private final Pneumatics m_pneumatics = new Pneumatics();

  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_helicopter = new Joystick(1);

  private final DriveCommand driveCommand = new DriveCommand(m_drive, m_joystick);

 // private final AlignCommand m_alignCommandCube = new AlignCommand(m_drive, m_vision, true);
  private final UsbCamera cm = CameraServer.startAutomaticCapture();
  

  /* Auto commands */

  private final ChargingStation m_chargingStation = new ChargingStation(m_drive, m_pulley);

  private final OnePieceChargingMobility m_onePieceC = new OnePieceChargingMobility(m_drive, m_pneumatics, m_pulley);

  private final OnePiece18 m_onePieceAuto = new OnePiece18(m_drive, m_pneumatics, m_pulley);
  
  private final AprilTagVision m_aprilTagVision = new AprilTagVision(m_drive);

  private final CubeThirdNode m_cubeThirdNode = new CubeThirdNode(m_pneumatics, m_pulley, m_drive, m_joystick);
  private final CubeSecondNode m_cubeSecondNode = new CubeSecondNode(m_pneumatics, m_pulley, m_joystick, m_drive);
  private final ConeThirdNode m_coneThirdNode = new ConeThirdNode(m_pneumatics, m_pulley, m_drive, m_joystick);
  private final ConeSecondNode m_coneSecondNode = new ConeSecondNode(m_pneumatics, m_pulley, m_drive);
  private final ChargingBrake m_chargingBrake = new ChargingBrake(m_drive);
  private final AlignCommand m_alignCommand = new AlignCommand(m_drive, m_vision, true);

  private final PIDDebugger m_pidDebugger = new PIDDebugger();

  private final OnlyCube m_onlyCube = new OnlyCube(m_pneumatics, m_pulley);

  
  
  private final SequentialCommandGroup m_twoPieceAuto = new SequentialCommandGroup(
    new CubeThirdNode(m_pneumatics, m_pulley, m_drive, m_joystick),
    new StraightDrive(m_drive, -1.5),
    new Turn180(m_drive, m_pulley),
    new StraightDrive(m_drive, -1.5),
    new AlignCommand(m_drive, m_vision, true),
    new InstantCommand(() -> {m_pneumatics.getIntakeSolenoid().close();}),
    new Turn180(m_drive, m_pulley),
    new StraightDrive(m_drive, 3.0),
    new CubeSecondNode(m_pneumatics, m_pulley, m_joystick, m_drive)
  );

  private final LinearPathFollower m_linearPathFollower = new LinearPathFollower(m_drive, m_aprilTagVision);
  private final DriveTaskFollower m_driveTaskFollower = new DriveTaskFollower(m_drive); 
  private final RotationTaskFollower m_rotationTaskFollower = new RotationTaskFollower(m_drive);
  private final StraightDrive m_straightDrive = new StraightDrive(m_drive, -0.5);
  private final Turn180 m_turn180 = new Turn180(m_drive, m_pulley);

  public RobotContainer() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    configureBindings();
    
    SmartDashboard.putData("Charging Station Balance", new ChargingStation(m_drive, m_pulley));
    SmartDashboard.putData("Cube Second", m_cubeSecondNode);
    SmartDashboard.putData("Cube Third", m_cubeThirdNode);
    SmartDashboard.putData("Cone Second", m_coneSecondNode);
    SmartDashboard.putData("Cone Third", m_coneThirdNode);
    SmartDashboard.putData("Charging Station Balance", new ChargingStation(m_drive, m_pulley));
    SmartDashboard.putData("AutoScore", new AutoScore(m_pneumatics, m_pulley));

    SmartDashboard.putNumber("Drive Task Distance", 0.0);
    SmartDashboard.putNumber("Rotation Task Degrees", 0.0);
    SmartDashboard.putNumber("Drive Task P", 0.6);
    SmartDashboard.putNumber("Drive Task D", 0.025);
    SmartDashboard.putNumber("Rotation Task P", 0.3);
    SmartDashboard.putNumber("Rotation Task D", 0.05);
    SmartDashboard.putBoolean("April Tag Path Follower", false);
    SmartDashboard.putNumber("Charging Volts", 7.5);

    m_pidDebugger.setPIDControllerToDashboard("Charging", 0.319, 0, 0.04);
    m_pidDebugger.setPIDControllerToDashboard("Align Linear", 0.15, 0, 0);
    m_pidDebugger.setPIDControllerToDashboard("Align Angular", 0.5, 0.01, 0.053);
    m_pidDebugger.setPIDControllerToDashboard("Straight Drive", 7, 0.2, 1.3);
    m_pidDebugger.setPIDControllerToDashboard("Turn", 0.07, 0.01, 0.013);
    
    SmartDashboard.putNumber("Brake P", 7.5);

  }

  private void configureBindings() {
    m_drive.setDefaultCommand(driveCommand);
  
    cm.setResolution(400, 280);

    JoystickButton button[] = {
      new JoystickButton(m_helicopter, 2), // toggle compressor
      new JoystickButton(m_helicopter, 5), // full open
      new JoystickButton(m_helicopter, 3), // full close
      new JoystickButton(m_helicopter, 1), // intake toggle
      new JoystickButton(m_joystick, 3), // slow sparks
      new JoystickButton(m_joystick, 1), // middle sparks
      new JoystickButton(m_joystick, 2), // fast sparks   
      new JoystickButton(m_joystick, 4), // brake mode toggle
      new JoystickButton(m_joystick, 5), // odometry button
      new JoystickButton(m_joystick, 6), // charging run
      new JoystickButton(m_joystick, 7), // charging stop
      new JoystickButton(m_joystick, 8) // turn 180
    };

    POVButton pov[] = {
      new POVButton(m_joystick, 0), // pulley reset
      new POVButton(m_joystick, 180), // pulley open
      new POVButton(m_joystick, 90), // pulley close
      new POVButton(m_joystick, 270), // turn 180
      new POVButton(m_joystick, 90), // pulley close
      new POVButton(m_helicopter, 0), // align command
      new POVButton(m_helicopter, 180) // align cancel
    };

    JoystickButton byHand[] = {
      new JoystickButton(m_helicopter, 10), //telescope toogle
      new JoystickButton(m_helicopter, 12), //arm toogle
      new JoystickButton(m_helicopter, 4), //pulley close
      new JoystickButton(m_helicopter, 6), //pulley open
      new JoystickButton(m_helicopter, 2), // charging brake
      new JoystickButton(m_helicopter,2), // path cancel
      new JoystickButton(m_helicopter, 11), // cube third
      new JoystickButton(m_helicopter, 9), // cube second
      new JoystickButton(m_helicopter, 7), // cone third
      new JoystickButton(m_helicopter, 8), // cone second
      new JoystickButton(m_helicopter, 3),
      new JoystickButton(m_helicopter, 5),
      new JoystickButton(m_helicopter, 8)
      
    };

    byHand[0].whileTrue(new InstantCommand(() -> m_pneumatics.getTelescopeSolenoid().toggle())); 
    byHand[1].whileTrue(new InstantCommand(() -> m_pneumatics.getArmSolenoid().toggle()));

    byHand[2].whileTrue(new InstantCommand(() -> m_pulley.openPulley())).whileFalse(new InstantCommand(() -> m_pulley.fixedPulley()));
    byHand[3].whileTrue(new InstantCommand(() -> m_pulley.closePulley())).whileFalse(new InstantCommand(() -> m_pulley.fixedPulley()));
    
    //byHand[4].onTrue(m_chargingBrake);
    byHand[6].onTrue(m_cubeThirdNode);
    byHand[7].onTrue(m_cubeSecondNode);
    byHand[8].onTrue(m_coneThirdNode);
    byHand[9].onTrue(m_coneSecondNode);
    //byHand[10].onTrue(m_linearPathFollower);
    //byHand[11].onTrue(new InstantCommand (() -> m_linearPathFollower.cancel()));
    byHand[12].onTrue(new InstantCommand(() -> m_cubeSecondNode.cancel()));
    byHand[12].onTrue(new InstantCommand(() -> m_cubeThirdNode.cancel()));
    byHand[12].onTrue(new InstantCommand(() -> m_coneSecondNode.cancel()));
    byHand[12].onTrue(new InstantCommand(() -> m_coneThirdNode.cancel()));
    

    button[0].whileTrue(new InstantCommand(() -> m_pneumatics.toggleCompressor()));
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
    button[11].onTrue(m_turn180);

    pov[0].onTrue(new InstantCommand(() -> m_pulley.reset()));

    pov[1].whileTrue(new InstantCommand(() -> m_pulley.openPulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));
    pov[2].whileTrue(new InstantCommand(() -> m_pulley.closePulley())).whileFalse(new InstantCommand(() -> m_pulley.stopPulley()));

    pov[3].onTrue(new InstantCommand(() -> m_turn180.cancel()));
    pov[5].onTrue(m_alignCommand);
    pov[6].onTrue(new InstantCommand( () -> m_alignCommand.cancel()));
  }

  public Command getAutonomousCommand() {
    return m_onePieceAuto;
  }

  public void testInit() {
    m_pneumatics.enableCompressor();
  }

  public void testPeriodic() {
    var volts = SmartDashboard.getNumber("Test Volts", 0);
    
    m_drive.tankDriveVolts(volts, volts);
  }

}