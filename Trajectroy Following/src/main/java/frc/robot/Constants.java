package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

    public static final double ksVolts = 0.9102; // 0.67319
    public static final double kvVoltSecondsPerMeter = 2.3049;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5; //1.3519
  
    public static final double kPDriveVel = 0; // 0.70093
  
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
    public static final double kP = 0.319;
    public static final double kI = 0.0;
    public static final double kD = 0.04;
  }

  public static class PulleyConstants{
    public static final int kA = 0;
    public static final int kB = 2;

    public static final double kFullCloseStateLength = 0.0;
    public static final double kFullOpenStateLength = 2650.0;
    public static final double kOnGroundStateLength = 1350.0;
    public static final double kArmOpenStateLength = 250.0;
    public static final double kTolerance = 50.0;

    public static final double kTime = 0.0;
    public static final double kTelescopeToIntake = 0.7;
  }
  
  public static class LinearPathConstants {
    public static final Translation2d kFieldLeftUp = new Translation2d(0.0, 0.0);
    public static final Translation2d kFieldRightDown = new Translation2d(0.0, 0.0);

    public static final Translation2d ID1 = new Translation2d(0.0, 0.0);
    public static final Translation2d ID2 = new Translation2d(0.0, 0.0);
    public static final Translation2d ID3 = new Translation2d(0.0, 0.0);
    public static final Translation2d ID4 = new Translation2d(0.0, 0.0);
    public static final Translation2d ID5 = new Translation2d(0.0, 0.0);
    public static final Translation2d ID6 = new Translation2d(0.0, 0.0);

    public static final double kAlignTolerance = 0.0;
    public static final double kAprilTagDistance = 0.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}