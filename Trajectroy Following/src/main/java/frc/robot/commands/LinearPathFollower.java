package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LinearPathConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.paths.P;
import frc.robot.paths.P.*;
import frc.robot.subsystems.Drive;

public class LinearPathFollower extends CommandBase {
    public class Task {
        public boolean isFinished;
        public void execute() {}

        public boolean isFinished() {
            return isFinished;
        }
    }

    public class DriveTask extends Task{  
        private final Command m_lineFollower;
        private final Pose2d m_pose;
        private final Pose2d m_endPose;
        private final Path m_line;

        public DriveTask(double distance, Pose2d pose) {
            m_drive.resetOdometry();

            isFinished = false;
            m_pose = pose;
            m_endPose = new Pose2d(new Translation2d(m_pose.getX() + distance, m_pose.getY()), m_pose.getRotation());

            m_line = new Path(m_pose, null, m_endPose, false);
            m_lineFollower = P.generateRamsete(m_drive, m_line);
        }

        public void execute() {
            m_lineFollower.execute();
            if(m_lineFollower.isFinished())
                this.isFinished = true;
        }
    }

    public class RotationTask extends Task{
        public boolean isFinished;   
        private final double m_degrees;
        private final double m_setPoint;
        private PIDController pid;   

        public RotationTask(double degrees) {
            isFinished = false;
            m_degrees = degrees;
            m_setPoint = m_drive.getAngle() + m_degrees;
            pid = new PIDController(LinearPathConstants.kP, LinearPathConstants.kI, LinearPathConstants.kD);
            pid.setTolerance(2.0);
        }

        public void execute() {
            var volts = pid.calculate(m_drive.getAngle(), m_setPoint);

            if(m_degrees < 0)
                m_drive.tankDriveVolts(-volts, volts); 
            else
                m_drive.tankDriveVolts(volts, -volts); 

            if(pid.atSetpoint())
                isFinished = true;
        }
    }

    private final Drive m_drive;
    private Pose2d m_pose;
    private int m_aprilTag;
    private Task[] m_taskSchedule;
    private boolean m_isStarted;
    private boolean m_isFinished;
    private int m_taskIterator;

    public LinearPathFollower(Drive drive) {
        m_drive = drive;
        m_taskIterator = 0;
        m_isStarted = false;
        m_isFinished = false;
        addRequirements(m_drive);
    }

    public void start(Pose2d pose, int aprilTag) {
        m_pose = pose;
        m_aprilTag = aprilTag;
        m_taskIterator = 0;
        m_isFinished = false;
        m_isStarted = true;
    }

    public Translation2d getAprilTagTranslation(int aprilTag) {
        switch(aprilTag) {
            case 1: return LinearPathConstants.ID1;
            case 2: return LinearPathConstants.ID2;
            case 3: return LinearPathConstants.ID3;
            case 4: return LinearPathConstants.ID4;
            case 5: return LinearPathConstants.ID5;
            case 6: return LinearPathConstants.ID6;
            default:
                return new Translation2d(0.0, 0.0);
        }
    }

    @Override
    public void initialize() {
        if(m_isStarted) {
            if (m_pose.getX() < LinearPathConstants.kFieldLeftUp.getX() && m_pose.getX() > LinearPathConstants.kFieldRightDown.getX() &&
                m_pose.getY() < LinearPathConstants.kFieldLeftUp.getY() && m_pose.getY() > LinearPathConstants.kFieldRightDown.getY()) {
                if(Math.abs(getAprilTagTranslation(m_aprilTag).getY() - m_pose.getY()) <= LinearPathConstants.kAlignTolerance) {
                    // ACROSS
                    m_taskSchedule = new Task[] {
                        new RotationTask(m_pose.getRotation().getDegrees()),
                        new DriveTask(m_pose.getX() - LinearPathConstants.kAprilTagDistance, m_pose)
                    };
                }
                else {
                    // INSIDE
                    m_taskSchedule = new Task[] {
                        new RotationTask(Math.atan(m_pose.getX() / m_pose.getY()) + m_pose.getRotation().getDegrees()),
                        new DriveTask(m_pose.getY(), m_pose),
                        new RotationTask(m_pose.getY() < 0 ? -90 : 90),
                        new DriveTask(m_pose.getX() - LinearPathConstants.kAprilTagDistance, m_pose)
                    };
                }
            } 
            else {
                // OUTSIDE
                m_taskSchedule = new Task[] {
                    new RotationTask(-(90 - (Math.atan(m_pose.getX() / m_pose.getY()) + m_pose.getRotation().getDegrees()))),
                    new DriveTask(m_pose.getX() - LinearPathConstants.kFieldLeftUp.getX(), m_pose),
                    new RotationTask(m_pose.getY() < 0 ? 90 : -90),
                    new DriveTask(m_pose.getY(), m_pose),
                    new RotationTask(m_pose.getY() < 0 ? -90 : 90),
                    new DriveTask(LinearPathConstants.kFieldLeftUp.getX() - LinearPathConstants.kAprilTagDistance, m_pose)
                };
            }
        }
        m_taskIterator = 0;
        m_isFinished = false;
    }

    @Override
    public void execute() {
        m_taskSchedule[m_taskIterator].execute();
        if(m_taskSchedule[m_taskIterator].isFinished) {
            if(m_taskSchedule.length == m_taskIterator)
                m_isFinished = true;
            else
                m_taskIterator++;
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
