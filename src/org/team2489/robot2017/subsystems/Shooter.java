package org.team2489.robot2017.subsystems;

import org.team2489.robot2017.Constants;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
	
	private static Shooter instance = new Shooter();

	private CANTalon mShooterFlywheelMaster;
	private CANTalon mShooterFlywheelSlave;
	public Shooter() {
		mShooterFlywheelMaster = new CANTalon(Constants.kShooterFlywheelMaster);
		mShooterFlywheelSlave = new CANTalon(Constants.kShooterFlywheelSlave);
		
		mShooterFlywheelMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		
		mShooterFlywheelSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		mShooterFlywheelSlave.set(Constants.kShooterFlywheelMaster);
		
		mShooterFlywheelMaster.setPID(Constants.kShooterVelocityKp, 
				Constants.kShooterVelocityKi, 
				Constants.kShooterVelocityKd, Constants.kShooterVelocityKf, 
				Constants.kShooterVelocityIZone, Constants.kShooterVelocityRampRate, 0);
		
		mShooterFlywheelMaster.setVoltageRampRate(Constants.kShooterVoltageRampRate);
		mShooterFlywheelSlave.setVoltageRampRate(Constants.kShooterVoltageRampRate);

		mShooterFlywheelMaster.enableBrakeMode(false);
		mShooterFlywheelSlave.enableBrakeMode(false);
	}
	
	public void setRPM(double rpm) {
		mShooterFlywheelMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
		mShooterFlywheelMaster.set(rpm);
	}
	
	public void setPower(double pwr) {
		mShooterFlywheelMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		mShooterFlywheelMaster.set(pwr);
	}
	
	public double getRPM() {
		return mShooterFlywheelMaster.getSpeed();
	}
	
	public synchronized double getTargetRPM() {
		return mShooterFlywheelMaster.getSetpoint();
	}

	// Mode must be speed.
	public synchronized boolean onTarget() {
		return (mShooterFlywheelMaster.getControlMode() == CANTalon.TalonControlMode.Voltage)
				&& Math.abs(getTargetRPM() - getRPM()) < Constants.kShooterAccurateShotTolerance;
	}
	
	public Shooter getInstance() {
		return instance;
	}

	// Subsystem
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("shooter_flywheel_on_target", onTarget());
		SmartDashboard.putNumber("shooter_flywheel_target_rpm", getTargetRPM());
		SmartDashboard.putNumber("shooter_flywheel_current_rpm", getRPM());
	}
	
	public void stop() {
		setPower(0.0);
	}
}
