package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public final class CTREConfigs {

    public static TalonFXConfiguration driveFXConfig;


    public CTREConfigs(){
        driveFXConfig = new TalonFXConfiguration();
        
        driveFXConfig.slot0.kP = Constants.Drive.kP;
        driveFXConfig.slot0.kI = Constants.Drive.kI;
        driveFXConfig.slot0.kD = Constants.Drive.kD;
        driveFXConfig.slot0.kF = Constants.Drive.kF;
    }
}