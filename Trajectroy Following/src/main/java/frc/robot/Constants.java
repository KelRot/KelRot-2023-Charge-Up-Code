package frc.robot;

import javax.sound.sampled.Line;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class EncoderConstants {
    public static final double kWheelC = 0.1525 * Math.PI;

    public static final double kDistancePerPulse = kWheelC * 1.0/1024.0;

    public static final int kLeftA = 5;
    public static final int kLeftB = 4;
    public static final int kRightA = 7;
    public static final int kRightB = 6;

  }                                   

  public static class TrajectoryConstants{

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double ksVolts = 0.66217; // 0.67319
    public static final double kvVoltSecondsPerMeter = 2.6307;
    public static final double kaVoltSecondsSquaredPerMeter = 1.3511; //1.3519
  
    public static final double kPDriveVel = 0.19378; // 0.70093
  
    public static final double kTrackwidthMeters = 0.57;

    public static final double kMaxVoltage = 10.0;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TrajectoryConstants.kTrackwidthMeters);
  };

  public static class DriveConstants {
    public static final double kMaxOutput = 10.0;

    public static final boolean isLeftInverted = false; 
    public static final boolean isRightInverted = true;

    public static final int kLeftBackMotorPort = 1;
    public static final int kLeftFrontMotorPort = 4;
    public static final int kRightBackMotorPort = 2;  
    public static final int kRightFrontMotorPort = 3;

    public static final double kChargingStationAngle = 15.0;
  }

  public static class PneumaticsConstants {
    public static final int[] kArmPins = {5, 2};
    public static final int[] kTelescopePins = {7, 0};
    public static final int[] kIntakePins = {1, 6};
  }

  public static class ChargingConstants {
    public static final double kP = 0.25;
    public static final double kI = 0.0;
    public static final double kD = 0.024; // 0.016

    public static final double kRequiredVoltage = -0.65; // required voltage to beat charging station
    public static final double kRequiredVoltageBack = 0.65; // required voltage to beat charging station
    public static final double kRequiredAngle = 13.5;
  }

  public static class PulleyConstants{
    public static final int kA = 0;
    public static final int kB = 2;

    public static final double kFullCloseStateLength = 0.0;
    public static final double kFullOpenStateLength = 3100.0;
    public static final double kOnGroundStateLength = 1740.0;
    public static final double kArmOpenStateLength = 1540.0;
    public static final double kCubeSecondStateLength = 350.0;
    public static final double kTolerance = 50.0;
    public static final double kMomentumTolerance = 0.0;

    public static final double kConeTwoState = 530.0;

    public static final double kTime = 0.0;
    public static final double kTelescopeToIntake = 0.7;
  }
  
  public static class LinearPathConstants {
    public static final Translation2d kFieldLeftDown = new Translation2d(14.60, 0.55);
    public static final Translation2d kFieldRightUp = new Translation2d(14.05, 5.04);

    public static final double kAlignTolerance = 0.15;
    public static final double kAprilTagDistance = 0.95;

    public static final double kP = 0.6;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
  
    public static final double kRotP = 0.6;
    public static final double kRotI = 0.0;
    public static final double kRotD = 0.1;
  }

  public static class AlignConstants {
    public static final double kAngularP = 0.05; // angular
    public static final double kAngularI = 0.0;
    public static final double kAngularD = 0.0;
    public static final double kAngularTolerance = 2.0; // deg

    public static final double kLinearP = LinearPathConstants.kP; // linear
    public static final double kLinearI = LinearPathConstants.kI;
    public static final double kLinearD = LinearPathConstants.kD;
    public static final double kLinearTolerance = 0.06; // m
  }

  public static class VisionConstants {
    public static final Transform3d robotToCam =
            new Transform3d(
                    new Translation3d(-0.41, 0.0, -0.39),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final String kUpperCamera = "Webcam_C170";
    public static final String kLowerCamera = "Webcam_C170";
  }

  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
  }

  public static class Turn180PID {
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.009;
  }
}