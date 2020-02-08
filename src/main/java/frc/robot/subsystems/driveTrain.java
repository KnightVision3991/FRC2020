/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.driveTrainCommand;

public class driveTrain extends SubsystemBase {



  //An array that stores all of our motors in an easy to iterate array rather than 6 different variables(Falcon 500 version) - Compiler will unwind forloops anyways
  /*private final WPI_TalonFX[] driveTrainMotors = {
    new WPI_TalonFX(0),
    new WPI_TalonFX(1),
    new WPI_TalonFX(2),

    new WPI_TalonFX(3),
    new WPI_TalonFX(4),
    new WPI_TalonFX(5),
  };*/


  //An array that stores all of our motors in an easy to iterate array rather than 6 different variables(Talon SRX version) - Compiler will unwind forloops anyways
  private final TalonFX[] driveTrainMotors = {
    new TalonFX(1),
    new TalonFX(2),
    new TalonFX(3),

    new TalonFX(4),
    new TalonFX(5),
    new TalonFX(6),
  };
  

  double leftTargetVelocity;
  double rightTargetVelocity; 
  public double PIDMultiplier = 6000 * 2048 / 600;
  SupplyCurrentLimitConfiguration driveLimitConfig = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.1);

  /**
   * Creates a new driveTrain.
   */
  public driveTrain() {


    for(int i = 0; i < 6; i++) {
      driveTrainMotors[i].configFactoryDefault();
      driveTrainMotors[i].configSupplyCurrentLimit(driveLimitConfig);

    }

    //Set each of the back two motors to follow the lead motor 
    driveTrainMotors[1].set(ControlMode.Follower, 0);
    driveTrainMotors[2].set(ControlMode.Follower, 0);

    driveTrainMotors[4].set(ControlMode.Follower, 3);
    driveTrainMotors[5].set(ControlMode.Follower, 3);

    driveTrainMotors[0].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    driveTrainMotors[3].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    driveTrainMotors[0].configNominalOutputForward(0, Constants.kTimeoutMs);
    driveTrainMotors[0].configNominalOutputReverse(0, Constants.kTimeoutMs);
    driveTrainMotors[0].configPeakOutputForward(1, Constants.kTimeoutMs);
    driveTrainMotors[0].configPeakOutputReverse(-1, Constants.kTimeoutMs);

    driveTrainMotors[0].config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity0.kF, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity0.kP, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity0.kI, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity0.kD, Constants.kTimeoutMs);


    driveTrainMotors[3].configNominalOutputForward(0, Constants.kTimeoutMs);
    driveTrainMotors[3].configNominalOutputReverse(0, Constants.kTimeoutMs);
    driveTrainMotors[3].configPeakOutputForward(1, Constants.kTimeoutMs);
    driveTrainMotors[3].configPeakOutputReverse(-1, Constants.kTimeoutMs);

    driveTrainMotors[3].config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity1.kF, Constants.kTimeoutMs);
		driveTrainMotors[3].config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity1.kP, Constants.kTimeoutMs);
		driveTrainMotors[3].config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity1.kI, Constants.kTimeoutMs);
    driveTrainMotors[3].config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity1.kD, Constants.kTimeoutMs);
    



    
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Side Encoder Values", driveTrainMotors[0].getSensorCollection().getIntegratedSensorVelocity()/2048 * 600);
    SmartDashboard.putNumber("Left Side Encoder Target", leftTargetVelocity);

    SmartDashboard.putNumber("Right Side Encoder Values", driveTrainMotors[3].getSensorCollection().getIntegratedSensorVelocity()/2048 * 600);
    SmartDashboard.putNumber("Right Side Encoder Target", rightTargetVelocity);


  }

  

  public void arcadeDrivePID(double throttle, double rot) {
    double leftPow = throttle + rot;
    double rightPow = throttle - rot;

    leftTargetVelocity = leftPow * 6000;
    rightTargetVelocity = rightPow * 6000;

    driveTrainMotors[0].set(ControlMode.Velocity, leftPow * PIDMultiplier);
    driveTrainMotors[3].set(ControlMode.Velocity, rightPow * PIDMultiplier);


  }
}
