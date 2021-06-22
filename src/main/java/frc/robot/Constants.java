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

import frc.lib.Controllers.TalonConstants;

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
		public static final double kP = 0.0;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kF = 0.0;

		public static final TalonConstants left1 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
		
		public static final TalonConstants left2 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
	
		public static final TalonConstants left3 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
	
		public static final TalonConstants right1 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
			
		public static final TalonConstants right2 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);
		
		public static final TalonConstants right3 = 
			new TalonConstants(1, CurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);

	}



    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;


	public final static Gains kGains_elevator = new Gains(0.25, 0.001, 20, .003, 300, 1);
	public final static Gains kGains_winch = new Gains(0.25, 0.001, 20, .003, 300, 1);
	public final static int[] elevatorPos = {0,200};
	

}
