package org.team2489.robot2017;

import edu.wpi.first.wpilibj.Joystick;

public class DriverStation {
	private static DriverStation instance = new DriverStation();
	
	public static DriverStation getInstance() {
		return instance;
	}
	
	private Joystick mLeftJoystick;
	private Joystick mRightJoystick;
	private Joystick mManipController;
	
	private DriverStation() {
		mLeftJoystick = new Joystick(0);
		mRightJoystick = new Joystick(1);
		mManipController = new Joystick(2);
	}
	
	public Joystick getLeftJoystick() {
		return mLeftJoystick;
	}
	
	public Joystick getRightJoystick() {
		return mRightJoystick;
	}
	
	public Joystick getManipController() {
		return mManipController;
	}
	
	public boolean getWantsHighShift() {
		return mRightJoystick.getTrigger();
	}
	
	public boolean getWantsLowShift() {
		return mLeftJoystick.getTrigger();
	}
	
	public boolean getWantsShoot() {
		return mManipController.getRawButton(0);
	}
	
	public boolean getWantsStopShooting() {
		return mManipController.getRawButton(1);
	}
}