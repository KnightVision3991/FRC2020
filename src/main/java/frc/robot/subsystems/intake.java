/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */
  private final TalonSRX intakeMotor = new TalonSRX(10);
  private double power;
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);

  public intake() {
    intakeMotor.configFactoryDefault();
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
