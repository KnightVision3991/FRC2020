package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
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
  DoubleSolenoid shifter = new DoubleSolenoid(11, 2, 4);

  private WPI_LazyTalonFX left1;
  private WPI_LazyTalonFX left2;
  private WPI_LazyTalonFX left3;
  private WPI_LazyTalonFX right1;
  private WPI_LazyTalonFX right2;
  private WPI_LazyTalonFX right3;

  private DifferentialDrive m_robotDrive;
  private DifferentialDriveOdometry m_robotDriveOdo;
  
  private SimpleMotorFeedforward driveFF;

  private PigeonIMU gyro;

  public enum shifterState {
    high, low
  } 

  private int currentNeutral = 0;

  private DriverStation ds;

  
  private double previousP = 0;

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

    gyro = new PigeonIMU(new TalonSRX(Constants.Drive.pigeonId));
    gyro.configFactoryDefault();

    m_robotDrive = new DifferentialDrive(left1, right1);
    m_robotDriveOdo = new DifferentialDriveOdometry(getYaw());
    m_robotDrive.setSafetyEnabled(false);
    m_robotDrive.setRightSideInverted(false);
    
    driveFF = new SimpleMotorFeedforward(Constants.Drive.driveKS / 12, Constants.Drive.driveKV / 12, Constants.Drive.driveKA / 12);

    ds = DriverStation.getInstance();

    
    SmartDashboard.putNumber("Drive p", 0);
    SmartDashboard.putNumber("Drive mps", 0);
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
    double leftDemand = Conversions.MPSToFalcon(leftSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);
    double rightDemand = Conversions.MPSToFalcon(rightSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);

    left1.set(ControlMode.Velocity, leftDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(leftDemand));
    right1.set(ControlMode.Velocity, rightDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(rightDemand));
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

  public void setNeutral(NeutralMode neutral){
    left1.setNeutralMode(neutral);
    left2.setNeutralMode(neutral);
    left3.setNeutralMode(neutral);
    right1.setNeutralMode(neutral);
    right2.setNeutralMode(neutral);
    right3.setNeutralMode(neutral);
  }


  @Override
  public void periodic() {
    m_robotDriveOdo.update(getYaw(), getPoseDouble()[0], getPoseDouble()[1]);
    SmartDashboard.putNumber("yaw", getYaw().getDegrees());
    SmartDashboard.putNumber("x", m_robotDriveOdo.getPoseMeters().getX());
    SmartDashboard.putNumber("y", m_robotDriveOdo.getPoseMeters().getY());

    if (ds.isEnabled() && currentNeutral == 1){
      setNeutral(NeutralMode.Brake);
      currentNeutral = 0;
    }
    
    if (ds.isDisabled() && currentNeutral == 0){
      setNeutral(NeutralMode.Coast);
      currentNeutral = 1;
    }

     /* Falcon Dashboard Setup*/
     NetworkTableInstance inst = NetworkTableInstance.getDefault();
     NetworkTable table = inst.getTable("Live_Dashboard");

     /* PoseEstimator Values */
     NetworkTableEntry robotX = table.getEntry("robotX");
     NetworkTableEntry robotY = table.getEntry("robotY");
     NetworkTableEntry robotHeading = table.getEntry("robotHeading");

     robotX.setDouble(Units.metersToFeet(m_robotDriveOdo.getPoseMeters().getX()));
     robotY.setDouble(Units.metersToFeet(m_robotDriveOdo.getPoseMeters().getY()));
     robotHeading.setDouble(m_robotDriveOdo.getPoseMeters().getRotation().getRadians());
     table.getEntry("isFollowingPath").setBoolean(true);
    
  //   double mps = SmartDashboard.getNumber("Drive mps", 0);
  //   setWheelState(mps, mps);

  //   if (previousP != SmartDashboard.getNumber("Drive p", 0)){
  //       previousP = SmartDashboard.getNumber("Drive p", 0);
  //       left1.config_kP(0, previousP);
  //       right1.config_kP(0, previousP);
  //   }

  //   SmartDashboard.putNumber("left drive", Conversions.falconToMPS(
  //       left1.getSelectedSensorVelocity(), 
  //       Constants.Drive.wheelCircumference, 
  //       Constants.Drive.gearRatio));

  //   SmartDashboard.putNumber("right drive", Conversions.falconToMPS(
  //       right1.getSelectedSensorVelocity(), 
  //       Constants.Drive.wheelCircumference, 
  //       Constants.Drive.gearRatio));
  }
}