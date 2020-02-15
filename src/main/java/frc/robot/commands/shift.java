/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.driveTrain.shifterState;

public class shift extends InstantCommand {
  private final driveTrain m_subsystem;
  private final shifterState m_shifterState;

  public shift(driveTrain subsystem, shifterState shifterState) {
    m_subsystem = subsystem;
    m_shifterState = shifterState;

    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.shift(m_shifterState);
  }
}
