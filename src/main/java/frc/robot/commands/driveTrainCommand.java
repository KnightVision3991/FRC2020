/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class driveTrainCommand extends CommandBase {


  private final driveTrain m_subsystem;

  double throttle;
  double rotation;

  
  public driveTrainCommand(driveTrain subsystem, double throttle, double rotation) {
    m_subsystem = subsystem;

    this.throttle = throttle;
    this.rotation = rotation;

    addRequirements(m_subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.arcadeDrivePID(throttle, rotation);

  }
}
