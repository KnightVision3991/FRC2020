/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.driveTrainCommand;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.setIntakePos;
import frc.robot.commands.shift;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.driveTrain.shifterState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final driveTrain driveTrain = new driveTrain();
  private final climber climber = new climber();
  private final intake intake = new intake();

  private final Command m_autoCommand = new exampleAuto(driveTrain, intake);


  private final XboxController gamepad = new XboxController(0);
    private final JoystickButton RB = new JoystickButton(gamepad, 6);
    private final JoystickButton LB = new JoystickButton(gamepad, 5);
    private final JoystickButton A = new JoystickButton(gamepad, 1);
    private final JoystickButton B = new JoystickButton(gamepad, 2);
    private final JoystickButton X = new JoystickButton(gamepad, 3);
    private final JoystickButton R = new JoystickButton(gamepad, 9);
    private final JoystickButton L = new JoystickButton(gamepad, 10);
  private final Joystick buttonBox = new Joystick(1);
    private final JoystickButton climbSwitch = new JoystickButton(buttonBox, 1);
    private final JoystickButton winchSwitch = new JoystickButton(buttonBox, 2);
    private final JoystickButton elevatorAdjustUp = new JoystickButton(buttonBox, 3);
    private final JoystickButton elevatorAdjustDown = new JoystickButton(buttonBox, 4);
    private final JoystickButton intakeIn = new JoystickButton(buttonBox, 5);
    private final JoystickButton intakeStop = new JoystickButton(buttonBox, 6);
    private final JoystickButton intakeOut = new JoystickButton(buttonBox, 7);
    private final JoystickButton armSwitch = new JoystickButton(buttonBox, 8);
    private final JoystickButton wheelSwitch = new JoystickButton(buttonBox, 9);
    private final JoystickButton wheelSpin = new JoystickButton(buttonBox, 10);
    private final JoystickButton wheelColor = new JoystickButton(buttonBox, 11);
    private final JoystickButton killPID = new JoystickButton(buttonBox, 12);
  


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    driveTrain.setDefaultCommand(new driveTrainCommand(driveTrain, 
    () -> gamepad.getTriggerAxis(Hand.kRight) - gamepad.getTriggerAxis(Hand.kLeft),
    () -> gamepad.getX(Hand.kLeft)));
    climber.setDefaultCommand(new elevatorCommand(climber));



    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    RB.whenActive(new shift(driveTrain, shifterState.high));
    LB.whenActive(new shift(driveTrain, shifterState.low));

    A.whileHeld(new IntakeControl(intake, 1));
    X.whileHeld(new IntakeControl(intake, -1));

    R.whenActive(new setIntakePos(intake, true));
    L.whenActive(new setIntakePos(intake, false));
    // intakeIn.whenActive(new intakeSet(intake, 1));
    // intakeStop.whenActive(new intakeSet(intake, 0));
    // intakeOut.whenActive(new intakeSet(intake, -1));
    armSwitch.whenActive(new setIntakePos(intake, true));
    armSwitch.whenInactive(new setIntakePos(intake, false));
    


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand.andThen(new InstantCommand(() -> driveTrain.drive(0, 0)));
  }
}
