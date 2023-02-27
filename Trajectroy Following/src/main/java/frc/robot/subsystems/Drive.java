package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;


public class Drive extends SubsystemBase {
  private final CANSparkMax m_leftBackMotor, m_leftFrontMotor, m_rightBackMotor, m_rightFrontMotor;


  private final Encoder m_leftEncoder, m_rightEncoder;

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final DifferentialDriveOdometry m_odometry;

  private final MotorControllerGroup m_leftMotorControllerGroup, m_rightMotorControllerGroup;

  private final DifferentialDrive m_drive;

  public final Field2d m_field = new Field2d();

  /**
  *Configures spark maxes: <p>
  *1: Restore factory defaults <p>
  *2: Invert <p>
  *3: Voltage compensations
  *   @param spark the sparkmax motor controller to be configured.
  */
  public void configureSpark(CANSparkMax spark){
    spark.restoreFactoryDefaults();
    spark.enableVoltageCompensation(6.0);
    spark.setIdleMode(IdleMode.kCoast);
  }
  
  public Drive() {
    m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorPort, MotorType.kBrushed);
    m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotorPort, MotorType.kBrushed);
    m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotorPort, MotorType.kBrushed);
    m_rightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotorPort, MotorType.kBrushed);

    configureSpark(m_leftFrontMotor);
    configureSpark(m_leftBackMotor);
    configureSpark(m_rightFrontMotor);
    configureSpark(m_rightBackMotor);

    m_leftMotorControllerGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    m_rightMotorControllerGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

    m_leftMotorControllerGroup.setInverted(DriveConstants.isLeftInverted);

    m_drive = new DifferentialDrive(m_leftMotorControllerGroup, m_rightMotorControllerGroup);

    m_leftEncoder = new Encoder(
      EncoderConstants.kLeftEncoderPorts[0],
      EncoderConstants.kLeftEncoderPorts[1],
      EncoderConstants.kLeftEncoderIsReversed
    );

    m_rightEncoder = new Encoder(
      EncoderConstants.kRightEncoderPorts[0],
      EncoderConstants.kRightEncoderPorts[1],
      EncoderConstants.kRightEncoderIsReversed
    );

    m_leftEncoder.setDistancePerPulse(EncoderConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(EncoderConstants.kDistancePerPulse);

    resetEncoders();
    resetGyro();

    m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance(),
      new Pose2d()
    );

    setMaxOutput(DriveConstants.kMaxOutput);

    debug();
  }

  /* Drive methods */

  public void curvatureDrive(Joystick js) {
    m_drive.curvatureDrive(js.getRawAxis(1), -js.getRawAxis(0) * 0.6, true);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {m_drive.tankDrive(leftVolts, rightVolts);}

  public void stopMotors() {m_drive.stopMotor();}

  public void setMaxOutput(double maxOutput) {m_drive.setMaxOutput(maxOutput);}

  public void feed() {m_drive.feed();}

  /* Encoder methods */

  public double[] getDistance() {return new double[] {m_leftEncoder.getDistance(), m_rightEncoder.getDistance()};}

  public void resetEncoders() {m_leftEncoder.reset(); m_rightEncoder.reset();}

  public double getAverageDistance() {return (getDistance()[0] + getDistance()[1]) / 2.0;}

  public void printDistance(){
    SmartDashboard.putNumber("Left encoder", getDistance()[0]);
    SmartDashboard.putNumber("Right encoder", getDistance()[1]);
    SmartDashboard.putData("Field", m_field);
  }

  /* Gyro methods */

  public double getAngle() {return m_gyro.getAngle();}

  public void resetGyro() {m_gyro.reset();}

  public void setGyroAxis(IMUAxis yaw_axis) {m_gyro.setYawAxis(yaw_axis);}

  public double getTurnRate() {return -m_gyro.getRate();}

  public void printAngle() {SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());}

  /* Odometry methods */

  public void resetOdometry() {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      new Pose2d()
    );
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      pose
    );
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public Pose2d getPose() {return m_odometry.getPoseMeters();}

  public void printPose(){
    var translation = m_odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("X", translation.getX());
    SmartDashboard.putNumber("Y", translation.getY());
  }

  /* Other methods */

  public void debug(){
    printPose();
    printAngle();
    printDistance();
  }

  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance()
    );
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {}
}
