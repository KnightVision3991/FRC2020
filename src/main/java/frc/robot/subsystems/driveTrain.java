/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    //Set each of the back two motors to follow the lead motor 
    /*driveTrainMotors[1].set(ControlMode.Follower, 0);
    driveTrainMotors[2].set(ControlMode.Follower, 0);

    driveTrainMotors[4].set(ControlMode.Follower, 3);
    driveTrainMotors[5].set(ControlMode.Follower, 3);*/

    for(int i = 0; i < 6; i++) {
      driveTrainMotors[i].configFactoryDefault();

      //Talon SRX
      //driveTrainMotors[i]

      //Talon FX
    }



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
