// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlignConstants;
import frc.robot.paths.P;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Drive;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AlignCommand extends CommandBase {
  private final Drive m_drive;
  private final Align m_align;
  private Command m_linearRamsete;

  final double linear_P = 0.1;
  final double linear_D = 0.0;
  private PIDController forwardController = new PIDController(linear_P, 0, linear_D);

  private PIDController m_angularPID;

  public AlignCommand(Drive drive, Align align) {
    m_align = align;
    m_drive = drive;
    m_angularPID = new PIDController(AlignConstants.kP, AlignConstants.kI, AlignConstants.kD);
    m_angularPID.setTolerance(AlignConstants.kTolerance);
    addRequirements(m_drive, m_align);
  }

  
  public void initialize() {
   /*  m_linearRamsete = P.generateRamsete(m_drive, 
      new P.Path(
        m_drive.getPose(), 
        null, 
        new Pose2d(
          m_drive.getPose().getX() + m_distance,
          m_drive.getPose().getY(),
          new Rotation2d()
        ), 
        false
      )
    );*/
  }

  @Override
  public void execute() {

    double forwardSpeed;
    double rotationSpeed;

    forwardSpeed = -m_joystick.getRawAxis(0);

    if (m_joystick.getPOV(90)) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(, 0);
        } else {
            // If we have no targets, stay still.
            rotationSpeed = 0;
        }
    } else {
        // Manual Driver Mode
        rotationSpeed = m_joystick.getRawAxis(0);        
    }

    m_drive.curvatureDrive(forwardSpeed, rotationSpeed, true);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
