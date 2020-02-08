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
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfigUtil;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
  TalonFXConfiguration configRight = new TalonFXConfiguration();
  TalonFXConfiguration configLeft = new TalonFXConfiguration();

  /**
   * Creates a new driveTrain.
   */
  public driveTrain() {

    configRight.nominalOutputForward = 0;
    configRight.nominalOutputReverse = 0;
    configRight.peakOutputForward = 1;
    configRight.peakOutputReverse = -1;
    configRight.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    configRight.slot0.kP = Constants.kGains_Velocity0.kP;
    configRight.slot0.kI = Constants.kGains_Velocity0.kI;
    configRight.slot0.kD = Constants.kGains_Velocity0.kD;
    configRight.slot0.kF = Constants.kGains_Velocity0.kF;

    configLeft.nominalOutputForward = 0;
    configLeft.nominalOutputReverse = 0;
    configLeft.peakOutputForward = 1;
    configLeft.peakOutputReverse = -1;
    configLeft.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    configLeft.slot0.kP = Constants.kGains_Velocity1.kP;
    configLeft.slot0.kI = Constants.kGains_Velocity1.kI;
    configLeft.slot0.kD = Constants.kGains_Velocity1.kD;
    configLeft.slot0.kF = Constants.kGains_Velocity1.kF;


    for(int i = 0; i < 3; i++) {
      driveTrainMotors[i].configFactoryDefault();
      driveTrainMotors[i].configSupplyCurrentLimit(driveLimitConfig);
      driveTrainMotors[i].configAllSettings(configRight);
    }
    for(int i = 3; i < 6; i++){
      driveTrainMotors[i].configFactoryDefault();
      driveTrainMotors[i].configSupplyCurrentLimit(driveLimitConfig);
      driveTrainMotors[i].configAllSettings(configLeft);
    }

    //Set each of the back two motors to follow the lead motor 
    driveTrainMotors[1].follow(driveTrainMotors[0]);
    driveTrainMotors[2].follow(driveTrainMotors[0]);

    driveTrainMotors[4].follow(driveTrainMotors[3]);
    driveTrainMotors[5].follow(driveTrainMotors[3]);  
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
