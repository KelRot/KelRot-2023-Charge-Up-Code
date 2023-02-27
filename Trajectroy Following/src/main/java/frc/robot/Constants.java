package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class EncoderConstants {
    public static final double kDistancePerPulse = 1.0/2048.0;

    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final boolean kLeftEncoderIsReversed = false;
    public static final int[] kRightEncoderPorts = {8, 7};
    public static final boolean kRightEncoderIsReversed = false;
  }                                   

  public static class TrajectoryConstants{
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double ksVolts = 0.63579; //90835
    public static final double kvVoltSecondsPerMeter = 2.6171;
    public static final double kaVoltSecondsSquaredPerMeter = 0.59256;
  
    public static final double kPDriveVel = 3.7176; // 3.7176
  
    public static final double kTrackwidthMeters = 0.56;

    public static final double kMaxVoltage = 10.0;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TrajectoryConstants.kTrackwidthMeters);
  };

  public static class DriveConstants {
    public static final double kMaxOutput = 12.0;

    public static final boolean isLeftInverted = true;

    public static final int kLeftBackMotorPort = 4;
    public static final int kLeftFrontMotorPort = 2;
    public static final int kRightBackMotorPort = 1;
    public static final int kRightFrontMotorPort = 3;
  }
}
