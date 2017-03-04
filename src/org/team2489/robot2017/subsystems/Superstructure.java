package org.team2489.robot2017.subsystems;

public class Superstructure extends Subsystem {
	public enum WantedFireState {
		STOP_FIRING,
		FIRE_NOW,
		FIRE_WHEN_READY
	}
	
	public enum WantedGearState {
		HOLD_GEAR,
		ROTATE,
		DEPLOY_WHEN_READY,
		DEPLOY_NOW
	}
	
	public enum WantedBallCirculationState {
		CIRCULATE_FORWARD, // Circulate the balls towards the hopper, no intake
		REVERSE // Deploy intake and flush balls from robot
	}
	
	private Shooter mShooter = new Shooter();
	private Hopper mHopper = new Hopper();
	
	public void stop() {
		mShooter.stop();
		mHopper.stop();
	}
	
	public void outputToSmartDashboard() {
		mShooter.outputToSmartDashboard();
		mHopper.outputToSmartDashboard();
	}
}
