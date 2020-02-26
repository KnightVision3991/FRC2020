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

import frc.robot.Constants;
import frc.robot.commands.elevatorCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  /**
   * Creates a new climber.
   */

  private final TalonSRX elevatorMotor = new TalonSRX(7);
  private final TalonSRX winchMotor = new TalonSRX(8);
  private final TalonSRX winchFollower = new TalonSRX(9);
  TalonSRXConfiguration elevatorConfig = new TalonSRXConfiguration();
  TalonSRXConfiguration winchConfig = new TalonSRXConfiguration();

  private int pos;

  public climber() {
    pos = 0;
    elevatorConfig.nominalOutputForward = 0;
    elevatorConfig.nominalOutputReverse = 0;
    elevatorConfig.peakOutputForward = 1;
    elevatorConfig.peakOutputReverse = -1;
    elevatorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    elevatorConfig.slot0.kF = Constants.kGains_elevator.kF;
    elevatorConfig.slot0.kP = Constants.kGains_elevator.kP;
    elevatorConfig.slot0.kI = Constants.kGains_elevator.kI;
    elevatorConfig.slot0.kD = Constants.kGains_elevator.kD;

    winchConfig.nominalOutputForward = 0;
    winchConfig.nominalOutputReverse = 0;
    winchConfig.peakOutputForward = 1;
    winchConfig.peakOutputReverse = -1;
    winchConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    winchConfig.slot0.kF = Constants.kGains_winch.kF;
    winchConfig.slot0.kP = Constants.kGains_winch.kP;
    winchConfig.slot0.kI = Constants.kGains_winch.kI;
    winchConfig.slot0.kD = Constants.kGains_winch.kD;


    elevatorMotor.configFactoryDefault();
    winchMotor.configFactoryDefault();
    winchFollower.configFactoryDefault();

    elevatorMotor.configAllSettings(elevatorConfig);
    winchMotor.configAllSettings(winchConfig);
    winchFollower.configAllSettings(winchConfig);


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
