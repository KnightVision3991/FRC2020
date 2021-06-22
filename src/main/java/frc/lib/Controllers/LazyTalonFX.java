package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Thin Falcon wrapper to make setup easier and reduce CAN bus overhead 
 * by skipping duplicate commands.
 */
public class LazyTalonFX extends TalonFX {
    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    /**
     * Config using talonFxConstants.
     * 
     * @param talonFxConstants
     */
    public LazyTalonFX(TalonConstants talonConstants) {
        super(talonConstants.deviceNumber);
        super.configFactoryDefault();
        super.configSupplyCurrentLimit(talonConstants.currentLimit);
        super.setNeutralMode(talonConstants.neutralMode);
        super.setInverted(talonConstants.invertType);
        super.setSelectedSensorPosition(0);
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != lastSet || mode != lastControlMode) {
            lastSet = value;
            lastControlMode = mode;
            super.set(mode, value);
        }
    }
    
}