package frc.robot.test_commands;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LinearPathConstants;
import frc.robot.commands.LinearPathFollower;
import frc.robot.paths.P;
import frc.robot.paths.P.Path;
import frc.robot.subsystems.Drive;

public class RotationTaskFollower extends CommandBase {
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

    public class RotationTask extends Task{  
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
    }

    public RotationTaskFollower(Drive drive) {
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
        m_drive.setNormalMode();
        double degrees = SmartDashboard.getNumber("Rotation Task Degrees", 0.0);
        SmartDashboard.putString("Rotation Task", "Initialized");
        m_taskSchedule = new Task[] { new RotationTask(degrees) };
        m_isFinished = false;
        m_isStarted = true;
    }

    @Override
    public void execute() {
        if(m_isStarted) {
            SmartDashboard.putString("Rotation Task", "Executing");
            m_taskSchedule[m_taskIterator].execute();
            SmartDashboard.putNumber("Rotation Task Iterator", m_taskIterator);
            SmartDashboard.putNumber("Rotation Task Length", m_taskSchedule.length);
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
        SmartDashboard.putString("Rotation Task", "Finished");
        m_isStarted = false;
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
