package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.setIntakePos;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.intake;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class exampleAuto extends SequentialCommandGroup {

  // An example trajectory.  All units in meters.
  private Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction 
        List.of(                              
          new Translation2d(3.2, -2)
        ), 
        new Pose2d(3.2, -3, new Rotation2d(0)), // End 3 meters straight ahead of where we started, facing forward
      Constants.AutoConstants.trajConfig);

  public exampleAuto(driveTrain m_robotDrive, intake m_robotIntake) {
    addRequirements(m_robotDrive);
    addRequirements(m_robotIntake);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
            m_robotDrive);

    addCommands(
      new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
      new InstantCommand(() -> m_robotDrive.setNeutral(NeutralMode.Brake)),
      new setIntakePos(m_robotIntake, false),
      new WaitCommand(3),
      new setIntakePos(m_robotIntake, true),

      ramseteCommand,
      new InstantCommand(() -> m_robotDrive.drive(0, 0)),

      new ParallelDeadlineGroup(
        new WaitCommand(2.0), 
        new IntakeControl(m_robotIntake, -1.0)
      ),
      new IntakeControl(m_robotIntake, 0)

    );

  }

}
