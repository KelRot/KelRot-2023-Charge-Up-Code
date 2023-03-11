package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
  private final Drive m_drive;
  private final Joystick m_joystick;

  public DriveCommand(Drive drive, Joystick js) {
    m_drive = drive;
    m_joystick = js;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.resetGyro();
    m_drive.resetEncoders();  
  }

  @Override
  public void execute() {
    m_drive.drive(m_joystick);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
