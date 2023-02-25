package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.Ramsete;
import frc.robot.subsystems.Drive;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final Drive m_drive = new Drive();

  private final Joystick m_joystick = new Joystick(0);

  private final DriveCommand driveCommand = new DriveCommand(m_drive, m_joystick);

  private final Ramsete m_ramsete = new Ramsete(m_drive);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(driveCommand);
    JoystickButton debug[] = {
        new JoystickButton(m_joystick, 1)
    };

    debug[0].whileTrue(new InstantCommand(() -> m_drive.resetOdometry()));
  }

  public Command getAutonomousCommand() {
    return m_ramsete.andThen(() -> m_drive.stopMotors());
  }
}
