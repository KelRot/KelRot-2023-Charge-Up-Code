package frc.robot.test_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Drive;

public class DriveTaskFollower extends CommandBase {
    private final Drive m_drive;
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
        private final PIDController m_pid;
        private final double m_startDistance;
        private final double m_setPoint;

        public DriveTask(double distance) {
            m_drive.resetOdometry(m_drive.getPose());
            m_pid = new PIDController(SmartDashboard.getNumber("Drive Task P", 0.2) , 0.0, SmartDashboard.getNumber("Drive Task D", 0.065));
            m_startDistance = m_drive.getAverageDistance();
            m_setPoint = (m_startDistance + distance) * 100;
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

        m_distance = SmartDashboard.getNumber("Drive Task Distance", 1.0);
        SmartDashboard.putString("Drive Task", "Initialized");
        m_taskSchedule = new Task[] { new DriveTask(m_distance) };
        m_isFinished = false;
        m_isStarted = true;
    }

    @Override
    public void execute() {
        if(m_isStarted) {
            SmartDashboard.putString("Drive Task", "Executing");
            m_taskSchedule[m_taskIterator].execute();
            if(m_taskSchedule[m_taskIterator].isFinished) {
                if(m_taskSchedule.length - 1  == m_taskIterator)
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
        SmartDashboard.putString("Drive Task", "Finished");
        m_isStarted = false;
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
