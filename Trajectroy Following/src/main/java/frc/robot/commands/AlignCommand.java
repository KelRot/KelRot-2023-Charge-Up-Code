// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignCommand extends CommandBase {
  private final Drive m_drive;
  private final Align m_vision;

  private PIDController m_angularPID, m_linearPID;

  private boolean m_aligned, m_piece, m_finished; // true if cube, false if cone

  public AlignCommand(Drive drive, Align vision, boolean piece) {
    m_vision = vision;
    m_drive = drive;
    m_piece = piece;

    m_angularPID = new PIDController(AlignConstants.kAngularP, AlignConstants.kAngularI, AlignConstants.kAngularD);
    m_angularPID.setTolerance(AlignConstants.kAngularTolerance);

    m_linearPID = new PIDController(AlignConstants.kLinearP, AlignConstants.kLinearI, AlignConstants.kLinearD);
    m_linearPID.setTolerance(AlignConstants.kLinearTolerance);

    addRequirements(m_drive, m_vision);
  }

  
  public void initialize() {
    m_drive.stopMotors();
    m_aligned = m_finished = false;
    m_vision.changePipeline(m_piece);
  }

  @Override
  public void execute() {
    if(!m_aligned){
      m_drive.curvatureDrive(
        0, 
        m_angularPID.calculate(m_vision.getAngle())
      );
      if(m_angularPID.atSetpoint()){
        m_aligned = true;
      }
    }else{
      m_drive.curvatureDrive(
        TrajectoryConstants.ksVolts + m_linearPID.calculate(m_vision.getDistance()), 
        0
      );
      if(m_linearPID.atSetpoint()){
        m_finished = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
