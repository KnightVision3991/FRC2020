/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.lib.Controllers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.commands.elevatorCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  /**
   * Creates a new climber.
   */

  private LazyTalonSRX elevatorMotor;
  private LazyTalonSRX winchMotor;
  private LazyTalonSRX winchFollower;

  private int pos;

  public climber() {
    elevatorMotor = new LazyTalonSRX(Constants.Climber.elevatorMotor);
    winchMotor = new LazyTalonSRX(Constants.Climber.winchMotor);
    winchFollower = new LazyTalonSRX(Constants.Climber.winchFollower);
    winchFollower.follow(winchMotor);
    
    CommandScheduler.getInstance().setDefaultCommand(this, new elevatorCommand(this));

  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void liftElevator(){
    // This method lifts elevator to place hook on bar
    elevatorMotor.set(ControlMode.Position, pos);
  }

  public void yeetWinch(){
    // This method activates winch 
    winchMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopWinch(){
    // This method deactivates winch
    winchMotor.set(ControlMode.PercentOutput, 0);
  }

  public int elevatorPosition(){
    return pos;
  }

  public void setElevatorPosition(int position){
    pos = position;
  }                 
}
