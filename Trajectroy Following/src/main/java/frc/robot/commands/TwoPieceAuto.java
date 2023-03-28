// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pulley;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Drive m_drive, Pulley m_pulley, Pneumatics m_pneumatics, Align m_vision, Joystick m_joystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CubeThirdNode(m_pneumatics, m_pulley, m_drive, m_joystick),
      new StraightDrive(m_drive, -1.5),
      new Turn180(m_drive, m_pulley),
      new StraightDrive(m_drive, -1.5),
      new AlignCommand(m_drive, m_vision, true),
      new InstantCommand(() -> {m_pneumatics.getIntakeSolenoid().close();}),
      new Turn180(m_drive, m_pulley),
      new StraightDrive(m_drive, 3.0),
      new CubeSecondNode(m_pneumatics, m_pulley, m_joystick, m_drive)
    );
  }
}
