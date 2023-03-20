package frc.robot.test_commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LinearPathFollower;
import frc.robot.paths.P;
import frc.robot.paths.P.Path;
import frc.robot.subsystems.Drive;

public class DriveTaskFollower extends CommandBase {
    private final Drive m_drive;
    private Pose2d m_pose;
    private double m_distance;
    private Task[] m_taskSchedule;
    private boolean m_isStarted;
    private boolean m_isFinished;
    private int m_taskIterator;

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
        private final Translation2d m_wayPoint;
        private final Path m_line;

        public DriveTask(double distance, Pose2d pose) {
            m_drive.resetOdometry();

            isFinished = false;
            m_pose = pose;
            m_endPose = new Pose2d(new Translation2d(m_pose.getX() + distance, m_pose.getY()), m_pose.getRotation());
            m_wayPoint = new Translation2d((m_pose.getX() + m_endPose.getX()) / 2, (m_pose.getY()));
            m_line = new Path(m_pose, Arrays.asList(m_wayPoint), m_endPose, false);
            m_lineFollower = P.generateRamsete(m_drive, m_line);
        }

        public void execute() {
            m_lineFollower.execute();
            if(m_lineFollower.isFinished()){
                isFinished = true;
            }
        }
    }

    public DriveTaskFollower(Drive drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    public void clearTaskSchedule() {
        m_taskSchedule = new Task[] {};
        m_taskIterator = 0;
    }

    @Override
    public void initialize() {
        clearTaskSchedule();
        m_pose = new Pose2d(
            SmartDashboard.getNumber("Drive Task X", 0.0), 
            SmartDashboard.getNumber("Drive Task Y", 0.0),
            Rotation2d.fromDegrees(m_drive.getAngle()));
        m_distance = SmartDashboard.getNumber("Drive Task Distance", 1.0);
        m_taskSchedule = new Task[] { new DriveTask(m_distance, m_pose) };
        m_isFinished = false;
        m_isStarted = true;
    }

    @Override
    public void execute() {
        if(m_isStarted) {
            m_taskSchedule[m_taskIterator].execute();
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
