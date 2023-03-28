// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDDebugger;
import frc.robot.Constants.AlignConstants;
import frc.robot.paths.P;
import frc.robot.subsystems.Drive;

public class StraightDrive extends CommandBase {
  private final Drive m_drive;
  private final double m_distance;
  private PIDController m_linearPID, m_angularPID;
  private final PIDDebugger m_pidDebugger;
  private boolean m_isFinished;
  /** Creates a new StraightDrive. */
  public StraightDrive(Drive drive, double distance) {
    m_drive = drive;
    m_distance = distance;
    m_pidDebugger = new PIDDebugger();
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially schedwuled.
  @Override
  public void initialize() {
    m_drive.stopMotors();
    m_drive.setIdleModeBrake(false);
    m_drive.setFastMode();

    m_drive.resetOdometry(m_drive.getPose());
    m_isFinished = false;

    m_linearPID = m_pidDebugger.getPIDControllerFromDashboard("Straight Drive");
    m_linearPID.setTolerance(AlignConstants.kLinearTolerance, 0.08);

    m_angularPID = new PIDController(0.02, 0.0, 0.0);

    m_linearPID.setSetpoint(m_distance);
    m_angularPID.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    m_drive.curvatureDrive(
      m_linearPID.calculate(m_drive.getAverageDistance()), 
      m_angularPID.calculate(-m_drive.getAngle())
    );
    */
    double volts = m_linearPID.calculate(m_drive.getAverageDistance());
    m_drive.tankDriveVolts(volts + m_angularPID.calculate(-m_drive.getAngle()), volts - m_angularPID.calculate(-m_drive.getAngle()));
  
    if(m_linearPID.atSetpoint()) {
      m_isFinished = true;
    }

    SmartDashboard.putBoolean("Straight Drive Is Finished", m_isFinished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
