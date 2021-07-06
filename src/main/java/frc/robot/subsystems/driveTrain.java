package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.lib.Controllers.WPI_LazyTalonFX;
import frc.lib.math.Boundaries;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class driveTrain extends SubsystemBase {
  double leftTargetVelocity;
  double rightTargetVelocity; 
  DoubleSolenoid shifter = new DoubleSolenoid(11,0,1);

  private WPI_LazyTalonFX left1;
  private WPI_LazyTalonFX left2;
  private WPI_LazyTalonFX left3;
  private WPI_LazyTalonFX right1;
  private WPI_LazyTalonFX right2;
  private WPI_LazyTalonFX right3;

  private DifferentialDrive m_robotDrive;
  private DifferentialDriveOdometry m_robotDriveOdo;

  private PigeonIMU gyro;

  public enum shifterState {
    high, low
  } 

  /**
   * Creates a new driveTrain.
   */
  public driveTrain() {
    left1 = new WPI_LazyTalonFX(Constants.Drive.left1);
    left2 = new WPI_LazyTalonFX(Constants.Drive.left2);
    left3 = new WPI_LazyTalonFX(Constants.Drive.left3);
    right1 = new WPI_LazyTalonFX(Constants.Drive.right1);
    right2 = new WPI_LazyTalonFX(Constants.Drive.right2);
    right3 = new WPI_LazyTalonFX(Constants.Drive.right3);

    left1.configAllSettings(CTREConfigs.driveFXConfig);
    left2.configAllSettings(CTREConfigs.driveFXConfig);
    left3.configAllSettings(CTREConfigs.driveFXConfig);
    right1.configAllSettings(CTREConfigs.driveFXConfig);
    right2.configAllSettings(CTREConfigs.driveFXConfig);
    right3.configAllSettings(CTREConfigs.driveFXConfig);

    left2.follow(left1);
    left3.follow(left1);

    right2.follow(right1);
    right3.follow(right1);

    gyro = new PigeonIMU(Constants.Drive.pigeonId);
    gyro.configFactoryDefault();

    m_robotDrive = new DifferentialDrive(left1, right1);
    m_robotDriveOdo = new DifferentialDriveOdometry(getYaw());
  }

  public void shift(shifterState shift){
    if(shift == shifterState.high){
      shifter.set(Value.kForward);
    }
    else{
      shifter.set(Value.kReverse);
    }
  }

  public void drive(double speed, double rotation){
    m_robotDrive.arcadeDrive(speed, rotation);
  }

  public void setWheelState(double leftSpeed, double rightSpeed){
    SimpleMotorFeedforward driveFF = Constants.Drive.driveFF;

    double leftDemand = Conversions.MPSToFalcon(leftSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);
    double rightDemand = Conversions.MPSToFalcon(rightSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);

    double leftFF = (driveFF.calculate(leftDemand) / 12.0); //Divide by 12 because CTRE is in percent not voltage
    double rightFF = (driveFF.calculate(rightDemand) / 12.0); //Divide by 12 because CTRE is in percent not voltage

    left1.set(ControlMode.Velocity, leftDemand, DemandType.ArbitraryFeedForward, leftFF);
    right1.set(ControlMode.Velocity, rightDemand, DemandType.ArbitraryFeedForward, rightFF);
}

  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    double yaw = Boundaries.to360Boundaries(ypr[0]);
    return Constants.Drive.invertGyro ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  public double[] getPoseDouble() {
    double leftMeters = 
      Conversions.falconToMeters(
        left1.getSelectedSensorPosition(), 
        Constants.Drive.wheelCircumference, 
        Constants.Drive.gearRatio
      );
      
    double rightMeters = 
      Conversions.falconToMeters(
        right1.getSelectedSensorPosition(), 
        Constants.Drive.wheelCircumference, 
        Constants.Drive.gearRatio
      );

    return new double[] {leftMeters, rightMeters};
  }

  public Pose2d getPose(){
    return m_robotDriveOdo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    left1.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
    m_robotDriveOdo.resetPosition(pose, getYaw());
}


  @Override
  public void periodic() {
    m_robotDriveOdo.update(getYaw(), getPoseDouble()[0], getPoseDouble()[1]);
    
  }
}
