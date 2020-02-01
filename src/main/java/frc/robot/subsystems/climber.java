/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.commands.elevatorCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  /**
   * Creates a new climber.
   */

  private final TalonSRX elevatorMotor = new TalonSRX(1);
  private final TalonSRX winchMotor = new TalonSRX(2);
  private final TalonSRX winchFollower = new TalonSRX(3);

  private final int pos;

  public climber() {
    pos = 0;
    elevatorMotor.configFactoryDefault();
    winchMotor.configFactoryDefault();
    winchFollower.configFactoryDefault();

    winchFollower.follow(winchMotor);

    elevatorMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    elevatorMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    elevatorMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_elevator.kF, Constants.kTimeoutMs);
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_elevator.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_elevator.kI, Constants.kTimeoutMs);
    elevatorMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_elevator.kD, Constants.kTimeoutMs);
    
    winchMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    winchMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    winchMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    winchMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    winchMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_winch.kF, Constants.kTimeoutMs);
		winchMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_winch.kP, Constants.kTimeoutMs);
		winchMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_winch.kI, Constants.kTimeoutMs);
    winchMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_winch.kD, Constants.kTimeoutMs);
    
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
    
  }
}
