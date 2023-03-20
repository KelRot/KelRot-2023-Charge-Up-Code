package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.TrajectoryConstants;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;


public class Drive extends SubsystemBase {
  private final WPI_VictorSPX m_leftBackMotor, m_leftFrontMotor, m_rightBackMotor, m_rightFrontMotor;

  private Encoder m_leftEncoder, m_rightEncoder;

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final DifferentialDriveOdometry m_odometry;

  private final MotorControllerGroup m_leftMotorControllerGroup, m_rightMotorControllerGroup;

  private final DifferentialDrive m_drive;

  private final DifferentialDriveKinematics m_kinematics;

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  public final Field2d m_field = new Field2d();

  private double kCompensation = 1.0;

  private boolean m_brake; // is brake mode

  /**
  *Configures victor maxes: <p>
  *1: Restore factory defaults <p>
  *2: Invert <p>
  *3: Voltage compensations
  *   @param victorspx the victor spx motor controller to be configured.
  */
  public void configureVictor(WPI_VictorSPX victor){
    victor.configFactoryDefault();
    victor.configVoltageCompSaturation(12.0);
    victor.enableVoltageCompensation(true);
  }

  public void configureEncoder(Encoder encoder, boolean isInverted){
    encoder.setDistancePerPulse(EncoderConstants.kDistancePerPulse);
    encoder.setReverseDirection(isInverted);
    encoder.reset();
  }
  
  public Drive() {
    m_leftFrontMotor = new WPI_VictorSPX(DriveConstants.kLeftFrontMotorPort);
    m_leftBackMotor = new WPI_VictorSPX(DriveConstants.kLeftBackMotorPort);
    m_rightFrontMotor = new WPI_VictorSPX(DriveConstants.kRightFrontMotorPort);
    m_rightBackMotor = new WPI_VictorSPX(DriveConstants.kRightBackMotorPort);

    configureVictor(m_leftFrontMotor);
    configureVictor(m_leftBackMotor);
    configureVictor(m_rightFrontMotor);
    configureVictor(m_rightBackMotor);

    m_leftMotorControllerGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    m_rightMotorControllerGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

    m_leftFrontMotor.setInverted(DriveConstants.isLeftInverted);
    m_leftBackMotor.setInverted(DriveConstants.isLeftInverted);

    m_rightFrontMotor.setInverted(DriveConstants.isRightInverted);
    m_rightBackMotor.setInverted(DriveConstants.isRightInverted);

    m_drive = new DifferentialDrive(m_leftMotorControllerGroup, m_rightMotorControllerGroup);

    m_kinematics = new DifferentialDriveKinematics(TrajectoryConstants.kTrackwidthMeters);

    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(getAngle()), 0.0, 0.0, new Pose2d());

    m_leftEncoder = new Encoder(EncoderConstants.kLeftA, EncoderConstants.kLeftB);
    m_rightEncoder = new Encoder(EncoderConstants.kRightA, EncoderConstants.kRightB);
    configureEncoder(m_leftEncoder, false);
    configureEncoder(m_rightEncoder, true);

    resetGyro();

    m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      getDistance()[0],
      getDistance()[1],
      new Pose2d()
    );

    setNormalMode();
    setVictorMode(NeutralMode.Coast);

    m_brake = false;

    debug();
  }

  /* Drive methods */

  public void drive(Joystick js) {
    m_drive.curvatureDrive(
      -js.getRawAxis(1) * kCompensation, 
      -js.getRawAxis(0) * 0.3 * kCompensation, 
      true
    );
  }

  public void curvatureDrive(double speed, double rot) {
    m_drive.curvatureDrive(speed * kCompensation, rot * kCompensation, true);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_drive.tankDrive(leftVolts / 12.0 * kCompensation, rightVolts / 12.0 * kCompensation);
    feed();
  }

  public void stopMotors() {m_drive.stopMotor();}


  public void feed() {m_drive.feed();}

  public void setMaxOutput(double kMaxSpeed){
    m_drive.setMaxOutput(kMaxSpeed);
    
  }

  public void setVoltageCompensation(double kVoltage) {
    kCompensation = kVoltage / 12.0;
  }

  public void setIdleModeBrake(boolean brake){
    m_brake = brake;
    if(m_brake == false){
      setVictorMode(NeutralMode.Coast);
    }else{
      setVictorMode(NeutralMode.Brake);
    }
  }

  public void changeIdleMode(){
    m_brake = !m_brake;
    if(m_brake == false){
      setVictorMode(NeutralMode.Coast);
    }else{
      setVictorMode(NeutralMode.Brake);
    }
  }

  public void setVictorMode(NeutralMode mode){
    m_leftBackMotor.setNeutralMode(mode);
    m_leftFrontMotor.setNeutralMode(mode);
    m_rightFrontMotor.setNeutralMode(mode);
    m_rightBackMotor.setNeutralMode(mode);
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

  public double getAngle() {return -m_gyro.getAngle();}

  public void resetGyro() {m_gyro.reset();}

  public void setGyroAxis(IMUAxis yaw_axis) {m_gyro.setYawAxis(yaw_axis);}

  public double getTurnRate() {return -m_gyro.getRate();}

  public void printAngle() {
    SmartDashboard.putNumber("Gyro Angle", -m_gyro.getAngle());
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

  /* Speed modes */

  public void setSlowMode() {
    setMaxOutput(4.0);
    setVoltageCompensation(4.0);
  }

  public void setNormalMode() {
    setMaxOutput(8.0);
    setVoltageCompensation(8.0);
  }

  public void setFastMode() {
    setMaxOutput(12.0);
    setVoltageCompensation(12.0);
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
    SmartDashboard.putNumber("KmaxVoltage", kCompensation);
    SmartDashboard.putNumber("Left voltage", m_leftBackMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right voltage", m_rightBackMotor.getMotorOutputVoltage());
  }

  @Override
  public void simulationPeriodic() {}

}