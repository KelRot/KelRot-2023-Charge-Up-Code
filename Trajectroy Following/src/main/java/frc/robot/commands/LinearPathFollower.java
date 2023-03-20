package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        public void execute() {}

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
            m_distance = distance;

            m_drive.resetOdometry(m_drive.getPose());
            m_pid = new PIDController(SmartDashboard.getNumber("Drive Task P", 0.55) , 0.0, SmartDashboard.getNumber("Drive Task D", 0.065));
            m_startDistance = m_drive.getAverageDistance();
            m_setPoint = (m_startDistance + m_distance) * 100;
            m_pid.setSetpoint(m_setPoint);
            m_pid.setTolerance(2, 4);
            
            isFinished = false;
            m_drive.setIdleModeBrake(true);
        }

        public void execute() {
            double volts = m_pid.calculate(m_drive.getAverageDistance() * 100) + TrajectoryConstants.ksVolts;
            m_drive.tankDriveVolts(volts, volts);
            
            if(m_pid.atSetpoint()) {
                isFinished = true;
                m_drive.setIdleModeBrake(false);
            }
            SmartDashboard.putBoolean("Drive Is Finished", isFinished);
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
            m_drive.resetOdometry();
            isFinished = false;
            m_degrees = degrees;
            m_setPoint = m_drive.getAngle() + m_degrees;
            pid = new PIDController(LinearPathConstants.kP, LinearPathConstants.kI, LinearPathConstants.kD);
            pid.setTolerance(2.0);
            pid.setSetpoint(m_setPoint);
            SmartDashboard.putNumber("Rotation Set Point", m_setPoint);
        }

        public void execute() {
            var volts = pid.calculate(m_drive.getAngle(), m_setPoint);
            SmartDashboard.putNumber("Rotation Volts", volts);
            SmartDashboard.putBoolean("Rotation Is Finished", isFinished);
            if(m_degrees < 0)
                m_drive.tankDriveVolts(-volts, volts); 
            else
                m_drive.tankDriveVolts(volts, -volts); 

            if(Math.abs(m_setPoint - m_drive.getAngle()) <= 2.0)
                isFinished = true;
        }

        public String debug() {
            return "Rotation Task: " + m_drive.getAngle() + " - " + m_setPoint;
        }
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
        clearTaskSchedule();
        m_drive.setNormalMode();
        if(SmartDashboard.getBoolean("April Tag Path Follower", false)) {
            if(m_vision.hasTargets) {
                m_pose = new Pose2d(m_vision.getPose().getX(), m_vision.getPose().getY(), Rotation2d.fromDegrees(-m_vision.getYaw()));
                m_aprilTag = m_vision.getId();
                m_isFinished = false;
                m_isStarted = true;
                try {
                    AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                    Pose3d m_aprilTagPose = fieldLayout.getTagPose(m_aprilTag).get();
                    if (m_pose.getX() < LinearPathConstants.kFieldLeftUp.getX() && m_pose.getX() > LinearPathConstants.kFieldRightDown.getX() &&
                        m_pose.getY() < LinearPathConstants.kFieldLeftUp.getY() && m_pose.getY() > LinearPathConstants.kFieldRightDown.getY()) {
                        if(Math.abs(m_aprilTagPose.getY() - m_pose.getY()) <= LinearPathConstants.kAlignTolerance) {
                            // ACROSS
                            m_taskSchedule = new Task[] {
                                new RotationTask(m_pose.getRotation().getDegrees()),
                                new DriveTask(m_pose.getX() - LinearPathConstants.kAprilTagDistance)
                            };
                        } else {
                            // INSIDE
                            m_taskSchedule = new Task[] {
                                new RotationTask(Math.atan(m_pose.getX() / m_pose.getY()) + m_pose.getRotation().getDegrees()),
                                new DriveTask(m_pose.getY()),
                                new RotationTask(m_pose.getY() < 0 ? -90 : 90),
                                new DriveTask(m_pose.getX() - LinearPathConstants.kAprilTagDistance)
                            };
                        }
                    } else {
                        // OUTSIDE
                        m_taskSchedule = new Task[] {
                            new RotationTask(-(90 - (Math.atan(m_pose.getX() / m_pose.getY()) + m_pose.getRotation().getDegrees()))),
                            new DriveTask(m_pose.getX() - LinearPathConstants.kFieldLeftUp.getX()),
                            new RotationTask(m_pose.getY() < 0 ? 90 : -90),
                            new DriveTask(m_pose.getY()),
                            new RotationTask(m_pose.getY() < 0 ? -90 : 90),
                            new DriveTask(LinearPathConstants.kFieldLeftUp.getX() - LinearPathConstants.kAprilTagDistance)
                        };
                    }
                } catch (IOException e) {
                    DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                    m_taskSchedule = new Task[] {};
                }
            }
        } else {
            m_taskSchedule = new Task[] {

            };
        }
    }

    @Override
    public void execute() {
        if(m_isStarted) {
            m_taskSchedule[m_taskIterator].execute();
            SmartDashboard.putString("Linear Path Follower", m_taskSchedule[m_taskIterator].debug());
            if(m_taskSchedule[m_taskIterator].isFinished) {
                if(m_taskSchedule.length == m_taskIterator)
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
        m_isStarted = false;
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
