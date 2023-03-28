// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ChargingBrake extends CommandBase {
  private final Drive m_drive;
  private PIDController m_pid;

  /** Creates a new ChargingBrake. */
  public ChargingBrake(Drive drive) {
    m_drive = drive;
    m_pid = new PIDController(0, 0, 0);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_drive.getPose());

    double kP = SmartDashboard.getNumber("Brake P", 0);
    //double kI = SmartDashboard.getNumber("Brake I", 0);
    //double kD = SmartDashboard.getNumber("Brake D", 0);
    m_pid = new PIDController(kP, 0, 0);

    m_pid.setSetpoint(0.0);
    m_pid.setTolerance(0.04, 0.05);
    m_drive.setIdleModeBrake(true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = m_drive.getAverageDistance();
    double volts = m_pid.calculate(error);
    m_drive.curvatureDrive(volts, 0.0);

    SmartDashboard.putBoolean("Is Stopped", m_pid.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
