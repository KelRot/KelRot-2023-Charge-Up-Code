package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class EncoderConstants {
    public static final double kDistancePerPulse = 1.0/512.0;

    public static final EncodingType kEncodingType = EncodingType.k1X;

    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final boolean kLeftEncoderIsReversed = false;
    public static final int[] kRightEncoderPorts = {8, 7};
    public static final boolean kRightEncoderIsReversed = false;
  }                                   

  public static class TrajectoryConstants{
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double ksVolts = 0.90835;
    public static final double kvVoltSecondsPerMeter = 2.5432;
    public static final double kaVoltSecondsSquaredPerMeter = 0.53875;
  
    public static final double kPDriveVel = 0;
  
    public static final double kTrackwidthMeters = 0.56;

    public static final double kMaxVoltage = 12.0;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TrajectoryConstants.kTrackwidthMeters);
  };

  public static class DriveConstants {
    public static final boolean isLeftInverted = true;

    public static final int kLeftBackMotorPort = 4;
    public static final int kLeftFrontMotorPort = 2;
    public static final int kRightBackMotorPort = 1;
    public static final int kRightFrontMotorPort = 3;
  }
}
