/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyTalonSRX;
import frc.robot.Constants;

public class intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */
  private LazyTalonSRX intakeMotor;
  private double power;
  private DoubleSolenoid intakeSolenoid;

  public intake() {
    intakeMotor = new LazyTalonSRX(Constants.Intake.intakeMotor);
    intakeSolenoid  = new DoubleSolenoid(Constants.Intake.pistonExtend, Constants.Intake.pistonRetract);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeSet(double pow){
    power = pow;
  }

  public void intakeWork(){
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void setIntakePos(boolean position){
    if(!position){
      intakeSolenoid.set(Value.kForward);
    }
    else{
      intakeSolenoid.set(Value.kReverse);
    }
  }

}
