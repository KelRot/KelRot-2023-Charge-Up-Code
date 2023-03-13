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

  private Encoder m_leftEncoder, m_rightEncoder;

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final DifferentialDriveOdometry m_odometry;

  private final MotorControllerGroup m_leftMotorControllerGroup, m_rightMotorControllerGroup;

  private final DifferentialDrive m_drive;

  public final Field2d m_field = new Field2d();

  private boolean m_brake; // is brake mode

  /**
  *Configures spark maxes: <p>
  *1: Restore factory defaults <p>
  *2: Invert <p>
  *3: Voltage compensations
  *   @param spark the sparkmax motor controller to be configured.
  */
  public void configureSpark(CANSparkMax spark){
    spark.restoreFactoryDefaults();
    spark.enableVoltageCompensation(12.0);
  }

  public void configureEncoder(Encoder encoder, boolean isInverted){
    encoder.setDistancePerPulse(EncoderConstants.kWheelC * EncoderConstants.kCountsPerRev / 4);
    encoder.setReverseDirection(isInverted);
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
    m_rightMotorControllerGroup.setInverted(DriveConstants.isRightInverted);

    m_drive = new DifferentialDrive(m_leftMotorControllerGroup, m_rightMotorControllerGroup);

    m_leftEncoder = new Encoder(EncoderConstants.kLeftA, EncoderConstants.kLeftB);
    m_rightEncoder = new Encoder(EncoderConstants.kRightA, EncoderConstants.kRightB);
    configureEncoder(m_leftEncoder, true);
    configureEncoder(m_rightEncoder, false);

    resetGyro();

    m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      getDistance()[0],
      getDistance()[1],
      new Pose2d()
    );

    setMaxOutput(DriveConstants.kMaxOutput);
    setSparkMode(IdleMode.kCoast);

    m_brake = false;

    debug();
  }

  /* Drive methods */

  public void drive(Joystick js) {
    m_drive.curvatureDrive(-js.getRawAxis(1), js.getRawAxis(0) * 0.5, true);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorControllerGroup.setVoltage(leftVolts);
    m_rightMotorControllerGroup.setVoltage(rightVolts);
    feed();
  }

  public void stopMotors() {m_drive.stopMotor();}


  public void feed() {m_drive.feed();}

  public void setMaxOutput(double kMaxSpeed){
    m_drive.setMaxOutput(kMaxSpeed);
    m_leftBackMotor.enableVoltageCompensation(kMaxSpeed);
    m_leftFrontMotor.enableVoltageCompensation(kMaxSpeed);
    m_rightFrontMotor.enableVoltageCompensation(kMaxSpeed);
    m_rightBackMotor.enableVoltageCompensation(kMaxSpeed);
  }

  public double getMaxOutput() {
    return m_leftFrontMotor.getVoltageCompensationNominalVoltage();
  }

  public void changeIdleMode(){
    m_brake = !m_brake;
    if(m_brake == false){
      setSparkMode(IdleMode.kCoast);
    }else{
      setSparkMode(IdleMode.kBrake);
    }
  }

  public void setSparkMode(IdleMode mode){
    m_leftBackMotor.setIdleMode(mode);
    m_leftFrontMotor.setIdleMode(mode);
    m_rightFrontMotor.setIdleMode(mode);
    m_rightBackMotor.setIdleMode(mode);
  }
  /* Encoder methods */
  public double[] getDistance() {return new double[] {m_leftEncoder.getDistance(), m_rightEncoder.getDistance()};}

  public void resetEncoders() {m_leftEncoder.reset(); m_rightEncoder.reset();}

  public double getAverageDistance() {return (getDistance()[0] + getDistance()[1]) / 2.0;}

  public void printDistance(){
    SmartDashboard.putNumber("Left encoder", getDistance()[0]);
    SmartDashboard.putNumber("Right encoder", getDistance()[1]);
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getRate());
    SmartDashboard.putData("Field", m_field);
  }

  /* Gyro methods */

  public double getAngle() {return m_gyro.getAngle();}

  public void resetGyro() {m_gyro.reset();}

  public void setGyroAxis(IMUAxis yaw_axis) {m_gyro.setYawAxis(yaw_axis);}

  public double getTurnRate() {return m_gyro.getRate();}

  public void printAngle() {
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
  }

  /* Odometry methods */

  public void resetOdometry() {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      getDistance()[0],
      getDistance()[1],
      new Pose2d()
    );
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      getDistance()[0],
      getDistance()[1],
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
      getDistance()[0],
      getDistance()[1]
    );
    m_field.setRobotPose(m_odometry.getPoseMeters());
    debug();
  }

  @Override
  public void simulationPeriodic() {}
}