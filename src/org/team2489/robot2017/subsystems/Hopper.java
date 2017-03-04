package org.team2489.robot2017.subsystems;
import org.team2489.robot2017.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;

public class Hopper extends Subsystem {
	public static Hopper instance = new Hopper();
	
	private CANTalon mHorizontalConveyor;
	private CANTalon mVerticalConveyor;
		
	public Hopper() {
		mHorizontalConveyor = new CANTalon(Constants.kHorizontalConveyor);
		mVerticalConveyor = new CANTalon(Constants.kVerticalConveyor);
						
		mHorizontalConveyor.setPID(Constants.kHorizontalConveyorCurrentKp, 
				Constants.kHorizontalConveyorCurrentKi, 
				Constants.kHorizontalConveyorCurrentKd, Constants.kHorizontalConveyorCurrentKf, 
				Constants.kHorizontalConveyorCurrentIZone, Constants.kHorizontalConveyorCurrentRampRate, 0);
		
		mVerticalConveyor.setPID(Constants.kVerticalConveyorCurrentKp, 
				Constants.kVerticalConveyorCurrentKi, 
				Constants.kVerticalConveyorCurrentKd, Constants.kVerticalConveyorCurrentKf, 
				Constants.kVerticalConveyorCurrentIZone, Constants.kVerticalConveyorCurrentRampRate, 0);

		mHorizontalConveyor.setVoltageRampRate(Constants.kHorizontalConveyorVoltageRampRate);
		mVerticalConveyor.setVoltageRampRate(Constants.kVerticalConveyorVoltageRampRate);

		mHorizontalConveyor.enableBrakeMode(false);
		mVerticalConveyor.enableBrakeMode(false);
	}
	
	public void setHorizontalCurrent(double curr) {
		mHorizontalConveyor.changeControlMode(CANTalon.TalonControlMode.Current);
		mHorizontalConveyor.set(curr);
	}
	
	public void setVerticalCurrent(double curr) {
		mVerticalConveyor.changeControlMode(CANTalon.TalonControlMode.Current);
		mVerticalConveyor.set(curr);
	}
	
	public void setHorizontalPower(double pwr) {
		mHorizontalConveyor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		mHorizontalConveyor.set(pwr);
	}
	
	public void setVerticalPower(double pwr) {
		mVerticalConveyor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		mVerticalConveyor.set(pwr);
	}
			
	public void stop() {
		setHorizontalPower(0);
		setVerticalPower(0);
	}
	
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("hopper_vertical_current", mVerticalConveyor.getOutputCurrent());
		SmartDashboard.putNumber("hopper_horizontal_current", mHorizontalConveyor.getOutputCurrent());
	}
}
