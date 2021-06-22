package frc.robot.commands;

import frc.robot.subsystems.driveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveTrainCommand extends CommandBase {
  private final driveTrain m_subsystem;

  DoubleSupplier throttle;
  DoubleSupplier rotation;

  
  public driveTrainCommand(driveTrain subsystem, DoubleSupplier throttle, DoubleSupplier rotation) {
    m_subsystem = subsystem;

    this.throttle = throttle;
    this.rotation = rotation;

    addRequirements(m_subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(throttle.getAsDouble(), rotation.getAsDouble());
  }
}
