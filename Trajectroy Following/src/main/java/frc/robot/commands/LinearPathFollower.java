package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LinearPathConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.Drive;

public class LinearPathFollower extends CommandBase {
    public class Task {
        public boolean isFinished;
        public boolean isInitialize;
        public void execute() {}

        public void initialize() {}

        public String debug() {
            return "";
        }

        public boolean isFinished() {
            return isFinished;
        }
    }

    public class DriveTask extends Task{  
        private final PIDController m_pid;
        private final double m_startDistance;
        private final double m_setPoint;
        private final double m_distance;

        public DriveTask(double distance) {
            //m_drive.resetOdometry(m_drive.getPose());
            m_distance = distance;
            m_pid = new PIDController(SmartDashboard.getNumber("Drive Task P", 0.6) , 0.0, SmartDashboard.getNumber("Drive Task D", 0.1));
            
            SmartDashboard.putNumber("Drive task d", distance);
            
            m_startDistance = m_drive.getAverageDistance();
            m_setPoint = (m_startDistance + m_distance) * 100;
            m_pid.setSetpoint(m_setPoint);
            m_pid.setTolerance(2, 4);
            
            isFinished = false;
            isInitialize = true;
            m_drive.setIdleModeBrake(true);
        }

        public void initialize() {
            m_drive.resetOdometry(m_drive.getPose());
            isInitialize = false;
        }

        public void execute() {
            double volts = m_pid.calculate(m_drive.getAverageDistance() * 100) + TrajectoryConstants.ksVolts;
            m_drive.tankDriveVolts(volts, volts);
            
            if(m_pid.atSetpoint()) {
                isFinished = true;
                m_drive.setIdleModeBrake(false);
            }
            if(isFinished) {
                end();
            }
            SmartDashboard.putBoolean("Drive Is Finished", isFinished);
        }

        public void end() {
        }

        public String debug() {
            return "Drive Task: " + m_drive.getAverageDistance() + " - " + m_setPoint;
        }
    }

    public class RotationTask extends Task {  
        private final double m_degrees;
        private final double m_setPoint;
        private PIDController pid;   

        public RotationTask(double degrees) {
            //m_drive.resetOdometry();
            m_degrees = degrees;
            m_setPoint = m_drive.getAngle() + m_degrees;
            pid = new PIDController(SmartDashboard.getNumber("Rotation Task P", 0.3), LinearPathConstants.kI, SmartDashboard.getNumber("Rotation Task D", 0.05));
            pid.setTolerance(2.0, 3.0);
            pid.setSetpoint(m_setPoint);

            isFinished = false;
            isInitialize = true;
        }

        public void initialize() {
            m_drive.resetOdometry();
            isInitialize = false;
        }

        public void execute() {
            var volts = pid.calculate(m_drive.getAngle(), m_setPoint);
            SmartDashboard.putBoolean("Rotation Is Finished", isFinished);
            m_drive.tankDriveVolts(volts, -volts); 

            if(pid.atSetpoint()) 
                isFinished = true;
            if(isFinished) {
                end();
            }
            SmartDashboard.putBoolean("Rotation Is Finished", isFinished);
        }

        public void end() {
        }

        public String debug() {
            return "Rotation Task: " + m_drive.getAngle() + " - " + m_setPoint;
        }
    }

    public boolean isBetweenTwoPoints(Pose2d obj, Translation2d rightUp, Translation2d leftDown){
        boolean isX = leftDown.getX() <= obj.getX() && obj.getX() <= rightUp.getX();
        boolean isY = leftDown.getY() <= obj.getY() && obj.getY() <= rightUp.getY();
        return (isX && isY);
    }

    private final Drive m_drive;
    private final AprilTagVision m_vision;
    private Pose2d m_pose;
    private int m_aprilTag;
    private Task[] m_taskSchedule;
    private boolean m_isStarted;
    private boolean m_isFinished;
    private int m_taskIterator;

    public LinearPathFollower(Drive drive, AprilTagVision vision) {
        m_drive = drive;
        m_vision = vision;
        m_taskIterator = 0;
        m_isStarted = false;
        m_isFinished = false;
        addRequirements(m_drive, m_vision);
    }

    public void clearTaskSchedule() {
        m_taskSchedule = new Task[] {};
        m_taskIterator = 0;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Linear Path Follower Command", "Initialized");
        clearTaskSchedule();
        m_drive.setNormalMode();
        if(SmartDashboard.getBoolean("April Tag Path Follower", false)) {
            if(m_vision.hasTargets) {
                m_pose = new Pose2d(
                    m_vision.getTagRelativePose().getX(),
                    m_vision.getTagRelativePose().getY(), 
                    Rotation2d.fromDegrees(-m_vision.getTagRelativePose().getRotation().getAngle())
                );

                m_aprilTag = m_vision.getId();
                m_isFinished = false;
                m_isStarted = true;
                try {
                    AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                    Pose3d m_aprilTagPose = fieldLayout.getTagPose(m_aprilTag).get();
                    if (isBetweenTwoPoints(m_pose, LinearPathConstants.kFieldLeftDown, LinearPathConstants.kFieldRightUp)) {
                        if(Math.abs(m_aprilTagPose.getY() - m_pose.getY()) <= LinearPathConstants.kAlignTolerance) {
                            // ACROSS
                            SmartDashboard.putBoolean("across", true);
                            m_taskSchedule = new Task[] {
                                new RotationTask(m_pose.getRotation().getDegrees()),
                                new DriveTask(m_aprilTagPose.getX() - LinearPathConstants.kAprilTagDistance - m_pose.getX())
                            };
                        } else {
                            // INSIDE
                            double r, alpha = m_pose.getRotation().getDegrees();
                            if(m_pose.getY() > m_aprilTagPose.getY()){
                                r = -90.0 - alpha;
                            }else{
                                r = 90.0 - alpha;
                            }
                            //may be reversed here
                            m_taskSchedule = new Task[] {
                                //new RotationTask(Math.atan(m_pose.getX() / m_pose.getY()) + m_pose.getRotation().getDegrees()),
                                new RotationTask(r),
                                new DriveTask(m_pose.getY() - m_aprilTagPose.getY()), // may be reversed
                                new RotationTask((m_pose.getY() > m_aprilTagPose.getY()) ? 90.0 : -90.0),
                                new DriveTask(m_aprilTagPose.getX() - LinearPathConstants.kAprilTagDistance - m_pose.getX())
                            };
                        }
                    } else {
                        // OUTSIDE
                        m_taskSchedule = new Task[] {
                            new RotationTask(m_pose.getRotation().getDegrees()),
                            new DriveTask(LinearPathConstants.kFieldLeftDown.getX() - m_pose.getX() + 0.10),
                            new RotationTask((m_pose.getY() > m_aprilTagPose.getY()) ? -90.0 : 90.0),
                            new DriveTask(m_pose.getY() - m_aprilTagPose.getY()),
                            new RotationTask((m_pose.getY() > m_aprilTagPose.getY()) ? 90.0 : -90.0),
                            new DriveTask(m_aprilTagPose.getX() - (LinearPathConstants.kFieldLeftDown.getX() + 0.10))
                        };
                    }
                } catch (IOException e) {
                    DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                    m_taskSchedule = new Task[] {};
                }
            }
        } else {
            m_taskSchedule = new Task[] {
                new DriveTask(1)
            };
            m_isFinished = false;
            m_isStarted = true;
        }
    }

    @Override
    public void execute() {
        if(m_isStarted) {
            SmartDashboard.putString("Linear Path Follower Command", "Executing");
            if(m_taskSchedule.length == 0){
                m_isFinished = true;
                return;
            }
            if(m_taskSchedule[m_taskIterator].isInitialize) {
                m_taskSchedule[m_taskIterator].initialize();
            } else {
                m_taskSchedule[m_taskIterator].execute();
            }
            
            SmartDashboard.putString("Linear Path Follower", m_taskSchedule[m_taskIterator].debug());
            if(m_taskSchedule[m_taskIterator].isFinished) {
                if(m_taskSchedule.length - 1 == m_taskIterator)
                    m_isFinished = true;
                else
                    m_taskIterator++;
            }
        } else {
            m_isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Linear Path Follower Command", "Finished");
        m_isStarted = false;
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
