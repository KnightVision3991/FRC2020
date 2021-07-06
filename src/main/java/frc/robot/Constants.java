/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.Controllers.TalonConstants;
import frc.lib.math.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class CurrentLimit {
		public static final SupplyCurrentLimitConfiguration supplyCurLim40 = 
			new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
			
		public static final SupplyCurrentLimitConfiguration supplyCurLim30 = 
			new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
	}

	public static final class Drive {

		public static final TalonConstants left1 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
		
		public static final TalonConstants left2 = 
			new TalonConstants(2, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
	
		public static final TalonConstants left3 = 
			new TalonConstants(3, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
	
		public static final TalonConstants right1 = 
			new TalonConstants(4, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
			
		public static final TalonConstants right2 = 
			new TalonConstants(5, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
		
		public static final TalonConstants right3 = 
			new TalonConstants(6, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);

		public static final int pigeonId = 1;
		public static final boolean invertGyro = false;

		public static final double gearRatio = (15.32 / 1.0); 
		public static final double wheelDiameter = Units.inchesToMeters(6.0);
		public static final double wheelCircumference = wheelDiameter * Math.PI;
		
		public static final double trackWidth = Units.inchesToMeters(22); //TODO
		public static final DifferentialDriveKinematics driveKinematics = 
			new DifferentialDriveKinematics(trackWidth);

		/* Drive Motor Feed Forward Characterization Values */
        public static final double driveKS = (0.648);
        public static final double driveKV = (2.09);
        public static final double driveKA = (0.286);
        public static final SimpleMotorFeedforward driveFF = 
            new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

		public static final double kP = 0; //TODO
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kF = 0;
	}

	public static final class Intake {
		public static final TalonConstants intakeMotor = 
			new TalonConstants(10, CurrentLimit.supplyCurLim30, NeutralMode.Brake, InvertType.None);
		public static final int pistonExtend = 2;
		public static final int pistonRetract = 4;
	}

	public static final class Climber {
		public static final TalonConstants elevatorMotor =
			new TalonConstants(7, CurrentLimit.supplyCurLim30, NeutralMode.Brake, InvertType.None);
		public static final TalonConstants winchMotor =
			new TalonConstants(8, CurrentLimit.supplyCurLim30, NeutralMode.Brake, InvertType.None);
		public static final TalonConstants winchFollower =
			new TalonConstants(9, CurrentLimit.supplyCurLim30, NeutralMode.Brake, InvertType.FollowMaster);

		public static final PIDGains elevatorGains = new PIDGains(0, 0, 0, 0);//todo
	}

	public static final class AutoConstants {
        public static final double maxSpeed = 3; //MPS TODO
        public static final double maxAcelleration = 3; //MPSS (meters per second squared) TODO

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Create a voltage constraint to ensure we don't accelerate too fast
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(Drive.driveFF, Drive.driveKinematics, 10);

        // Config for Trajectory Generation
        public static final TrajectoryConfig trajConfig =
            new TrajectoryConfig(maxSpeed, maxAcelleration)
                .setKinematics(Constants.Drive.driveKinematics)
                .addConstraint(Constants.AutoConstants.autoVoltageConstraint);
    }

}
