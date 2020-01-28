/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private final WPI_TalonSRX[] driveTrainMotors = {
    new WPI_TalonSRX(0),
    new WPI_TalonSRX(1),
    new WPI_TalonSRX(2),

    new WPI_TalonSRX(3),
    new WPI_TalonSRX(4),
    new WPI_TalonSRX(5),
  };

  /**
   * Creates a new driveTrain.
   */
  public driveTrain() {


    for(int i = 0; i < 6; i++) {
      driveTrainMotors[i].configFactoryDefault();

      //Talon SRX
      //driveTrainMotors[i]

      //Talon FX
    }

    //Set each of the back two motors to follow the lead motor 
    driveTrainMotors[1].set(ControlMode.Follower, 0);
    driveTrainMotors[2].set(ControlMode.Follower, 0);

    driveTrainMotors[4].set(ControlMode.Follower, 3);
    driveTrainMotors[5].set(ControlMode.Follower, 3);

    driveTrainMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    driveTrainMotors[3].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);


    driveTrainMotors[0].configNominalOutputForward(0, Constants.kTimeoutMs);
    driveTrainMotors[0].configNominalOutputReverse(0, Constants.kTimeoutMs);
    driveTrainMotors[0].configPeakOutputForward(1, Constants.kTimeoutMs);
    driveTrainMotors[0].configPeakOutputReverse(-1, Constants.kTimeoutMs);

    driveTrainMotors[0].config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		driveTrainMotors[0].config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);


    driveTrainMotors[3].configNominalOutputForward(0, Constants.kTimeoutMs);
    driveTrainMotors[3].configNominalOutputReverse(0, Constants.kTimeoutMs);
    driveTrainMotors[3].configPeakOutputForward(1, Constants.kTimeoutMs);
    driveTrainMotors[3].configPeakOutputReverse(-1, Constants.kTimeoutMs);

    driveTrainMotors[3].config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		driveTrainMotors[3].config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		driveTrainMotors[3].config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		driveTrainMotors[3].config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
