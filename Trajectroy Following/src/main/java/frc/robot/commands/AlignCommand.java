package frc.robot.commands;

import frc.robot.PIDDebugger;
import frc.robot.Constants.AlignConstants;
import frc.robot.paths.P;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignCommand extends CommandBase {
	private final Drive m_drive;
	private final Align m_vision;

	private PIDController m_angularPID, m_linearPID;
	private final PIDDebugger m_pidDebugger;

	private double m_cubeSetpoint = -21.0, m_coneSetpoint = -18.0;
	private boolean m_aligned, m_isCube, m_isFinished, m_moved; // true if cube, false if
																							// cone

	public AlignCommand(Drive drive, Align vision, boolean isCube) {
		m_vision = vision;
		m_vision.changePipeline(m_isCube);

		m_drive = drive;
		m_pidDebugger = new PIDDebugger();
		m_isCube = isCube;

		addRequirements(m_drive, m_vision);
	}

	public double getSetpoint() {
		return m_isCube ? m_cubeSetpoint : m_coneSetpoint;
	}

	public void initialize() {
		m_drive.stopMotors();
		m_drive.setFastMode();
		m_drive.setIdleModeBrake(false);
		m_moved = m_aligned = m_isFinished = false;

		SmartDashboard.putBoolean("Align Rotation Finished", m_aligned);
		SmartDashboard.putBoolean("Align Linear Finished", m_moved);

		SmartDashboard.putString("Align State", "Started");

		m_angularPID = m_pidDebugger.getPIDControllerFromDashboard("Align Angular");
		m_angularPID.setTolerance(AlignConstants.kAngularTolerance, 3.0);

		m_linearPID = m_pidDebugger.getPIDControllerFromDashboard("Align Linear");
		m_linearPID.setTolerance(AlignConstants.kLinearTolerance, 4.0);

		if (m_vision.hasFoundAnyTarget()) {
			double yaw = m_vision.getAngle();

			m_linearPID.setSetpoint(getSetpoint());
			m_angularPID.setSetpoint(-yaw);

			m_drive.resetOdometry(m_drive.getPose());
		} else {
			m_isFinished = true;
		}
	}

	@Override
	public void execute() {
		double leftVolts, rightVolts;
		if (!m_aligned) {
			rightVolts = m_angularPID.calculate(-m_drive.getAngle());
			leftVolts = -rightVolts;
			if (m_angularPID.atSetpoint()) {
				m_aligned = true;
				m_drive.resetOdometry(m_drive.getPose());
			}
		} 
		else {
			m_isFinished = true;
			leftVolts = 0;
			rightVolts = 0;
			/*
			double pitch = m_vision.getPitch();
			if (pitch >= getSetpoint() + 5.0) {
				leftVolts = 3.0;
				rightVolts = 3.0;
				// m_drive.curvatureDrive(0.3, m_angularPID.calculate(-m_drive.getAngle()));
			} 
			else {
				/*
				 * m_drive.curvatureDrive(
				 * -m_linearPID.calculate(m_vision.getPitch()),
				 * m_angularPID.calculate(-m_drive.getAngle())
				 * );
				 */ /*
				rightVolts = -m_linearPID.calculate(pitch);
				leftVolts = rightVolts;
			}

			if (m_linearPID.atSetpoint() || m_vision.hasTarget() == false) {
				m_moved = true;
				m_isFinished = true;
			}
			*/
		}

		m_drive.tankDriveVolts(leftVolts, rightVolts);

		SmartDashboard.putBoolean("Align Linear Finished", m_moved);
		SmartDashboard.putBoolean("Align Rotation Finished", m_aligned);
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setNormalMode();
		m_drive.setIdleModeBrake(false);
		m_drive.stopMotors();
		SmartDashboard.putString("Align State", "Finished");
	}

	@Override
	public boolean isFinished() {
		return m_isFinished;
	}
}
