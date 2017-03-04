package org.team2489.robot2017.subsystems;

import org.team2489.robot2017.Constants;
import org.team2489.robot2017.RobotState;
import org.team2489.robot2017.Kinematics;
import org.team2489.robot2017.loops.Loop;
import org.team2489.robot2017.utils.AdaptivePurePursuitController;
import org.team2489.robot2017.utils.Path;
import org.team2489.robot2017.utils.RigidTransform2d;
import org.team2489.robot2017.utils.Rotation2d;
import org.team2489.robot2017.utils.SynchronousPID;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;



public class Drivetrain extends Subsystem {
	public enum DrivetrainState {	
		OPEN_LOOP, LOCK, VELOCITY_CLOSED_LOOP, PATH_FOLLOWING_CONTROL, VELOCITY_HEADING_CLOSED_LOOP
	}
	
	private static Drivetrain instance = new Drivetrain();
	
	private final CANTalon mLeftDriveMaster, mLeftDriveSlave, mRightDriveMaster, mRightDriveSlave;
	
	private DoubleSolenoid mShifter;
	
	private AHRS mGyro = new AHRS(SerialPort.Port.kMXP);
	
	private AdaptivePurePursuitController mPathFollowingController;
	private VelocityHeadingSetpoint mVelocityHeadingSetpoint;
	
	private boolean isBreak;
	
	private DrivetrainState state;
	
	private SynchronousPID mVelocityHeadingPid;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            driveWithPower(0,0);
            mPathFollowingController = null;
            setBreak(false);
        }

        @Override
        public void onLoop() {
            synchronized (Drivetrain.this) {
                switch (state) {
                case OPEN_LOOP:
                    return;
                case LOCK:
                    return;
                case VELOCITY_CLOSED_LOOP:
                    // Talons are updating the control loop state
                    return;
                case VELOCITY_HEADING_CLOSED_LOOP:
                	return;
                case PATH_FOLLOWING_CONTROL:
                    updatePathFollower();
                    if (isFinishedPath()) {
                        stop();
                    }
                    break;
                default:
                    System.out.println("Unexpected drive control state: " + state);
                    break;
                }
            }
        }

        @Override
        public void onStop() {
        	stop();
        }
    };

	
	
	public static Drivetrain getInstance() {
		return instance;
	}
	
	public Drivetrain() {
		state = DrivetrainState.OPEN_LOOP;

		mShifter = new DoubleSolenoid(Constants.kShifterA,Constants.kShifterB);
		mLeftDriveMaster = new CANTalon(Constants.kLeftDriveMaster);
		mLeftDriveSlave = new CANTalon(Constants.kLeftDriveSlave);
		
		mLeftDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		
		mLeftDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		mLeftDriveSlave.set(Constants.kLeftDriveMaster);
		
		mRightDriveMaster = new CANTalon(Constants.kRightDriveMaster);
		mRightDriveSlave = new CANTalon(Constants.kRightDriveSlave);
		mRightDriveMaster.setInverted(true);
//		mRightDriveMaster.reverseSensor(true);

		mRightDriveSlave.setInverted(true);

		mRightDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);

		mRightDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		mRightDriveSlave.set(Constants.kRightDriveMaster);
		
//		mRightDriveMaster.reverseOutput(true);
//		mRightDriveSlave.reverseOutput(true);
		
		mRightDriveMaster.setPID(Constants.kDriveVelocityKp, 
				Constants.kDriveVelocityKi, 
				Constants.kDriveVelocityKd, Constants.kDriveVelocityKf, 
				Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate, 0);
		
		mLeftDriveMaster.setPID(Constants.kDriveVelocityKp, 
				Constants.kDriveVelocityKi, 
				Constants.kDriveVelocityKd, Constants.kDriveVelocityKf, 
				Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate, 0);
		
		mLeftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		mRightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		
		mVelocityHeadingPid = new SynchronousPID(Constants.kVelocityHeadingKp, Constants.kVelocityHeadingKi, 
				Constants.kVelocityHeadingKd);
		mVelocityHeadingPid.setOutputRange(-30, 30);
		
		setHighGear(true);
	}
	
	public synchronized void setLock() {
		if (state != DrivetrainState.LOCK) {
			mRightDriveMaster.setProfile(Constants.kDriveStaySlot);
			mRightDriveMaster.changeControlMode(CANTalon.TalonControlMode.Position);
			mRightDriveMaster.setAllowableClosedLoopErr(Constants.kDriveStayAllowedError);
			mRightDriveMaster.set(mRightDriveMaster.getPosition());
			
			mLeftDriveMaster.setProfile(Constants.kDriveStaySlot);
			mLeftDriveMaster.changeControlMode(CANTalon.TalonControlMode.Position);
			mLeftDriveMaster.setAllowableClosedLoopErr(Constants.kDriveStayAllowedError);
			mLeftDriveMaster.set(mRightDriveMaster.getPosition());
			
			setBreak(true);
			setHighGear(false);
		}
		
	} 
	
	public void driveWithPower(double leftPwr, double rightPwr) {
		configureForPowerControl();
		
		mRightDriveMaster.set(rightPwr);
		mLeftDriveMaster.set(leftPwr);
	}
	
	// Left and Right Velocity in in/s
	public void driveWithVelocity(double leftVelocity, double rightVelocity) {
		configureForVelocityControl();
		state = DrivetrainState.VELOCITY_CLOSED_LOOP;
		
		mRightDriveMaster.set(driveInchesPerSecondToRPM(rightVelocity));
		mLeftDriveMaster.set(driveInchesPerSecondToRPM(leftVelocity));
	}
	
	public double getLeftVelocity() {
		return mLeftDriveMaster.getSpeed();
	}
	
	public double getRightVelocity() {
		return mRightDriveMaster.getSpeed();
	}
		
	public void setHighGear(boolean isHigh) {
		if(isHigh) {
			mShifter.set(DoubleSolenoid.Value.kForward);
		} else {
			mShifter.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("drive_velocity_left", getLeftVelocityInchesPerSec());
		SmartDashboard.putNumber("drive_velocity_right", getRightVelocityInchesPerSec());
		SmartDashboard.putNumber("drive_distance_left", getLeftDistanceInches());
		SmartDashboard.putNumber("drive_distance_right", getRightDistanceInches());
		SmartDashboard.putNumber("drive_gyro_angle", mGyro.getAngle());
	}
	
	public void stop() {
        driveWithPower(0,0);
	}
	
	public Loop getLoop() {
		return mLoop;
	}
	
	public boolean getBreak() {
		return isBreak;
	}
	
	private void configureForPowerControl() {
		if (state != DrivetrainState.OPEN_LOOP) {
			mLeftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			mRightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		state = DrivetrainState.OPEN_LOOP;
	}
	
	private void configureForVelocityControl() {
		if (state == DrivetrainState.VELOCITY_CLOSED_LOOP || 
				state == DrivetrainState.PATH_FOLLOWING_CONTROL || 
				state == DrivetrainState.VELOCITY_HEADING_CLOSED_LOOP ) {
			return;
		}
		
		mLeftDriveMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
		mLeftDriveMaster.setProfile(Constants.kDriveVelocitySlot);
		mLeftDriveMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowedError);
		
		mRightDriveMaster.changeControlMode(CANTalon.TalonControlMode.Speed);	
		mRightDriveMaster.setProfile(Constants.kDriveVelocitySlot);
		mRightDriveMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowedError);
	}
	
	private void setBreak(boolean brakeMode) {
		isBreak = brakeMode;
		mLeftDriveMaster.enableBrakeMode(brakeMode);
		mRightDriveMaster.enableBrakeMode(brakeMode);
		mLeftDriveSlave.enableBrakeMode(brakeMode);
		mRightDriveSlave.enableBrakeMode(brakeMode);
	}
	
	public boolean isFinishedPath() {
		return (state != DrivetrainState.PATH_FOLLOWING_CONTROL) ||
				mPathFollowingController.isDone();
	}
	
	public void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = mPathFollowingController.update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kMaxVelocity) {
            double scaling = Constants.kMaxVelocity / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        driveWithVelocity(setpoint.left, setpoint.right);
	}
	
	public void followPath(Path p, boolean rev) {
		if (state != DrivetrainState.PATH_FOLLOWING_CONTROL) {
			configureForVelocityControl();
			state = DrivetrainState.PATH_FOLLOWING_CONTROL;
		}
		
		mPathFollowingController = new AdaptivePurePursuitController(Constants.kPathLookahead, 
				Constants.kMaxAccel, Constants.kLooperDt, p, rev, Constants.kPathTolerance);
		
		updatePathFollower();
	}
	
	public void updateVelocityHeadingSetpoint() {
		Rotation2d currentAngle = getGyroAngle();
		
		double prevHeadingErrorDeg = mVelocityHeadingSetpoint.getHeading().rotateBy(currentAngle.inverse()).getDegrees();
		double dv = mVelocityHeadingPid.calculate(prevHeadingErrorDeg);
		
		driveWithVelocity(mVelocityHeadingSetpoint.getLeftSpeed() + dv / 2, mVelocityHeadingSetpoint.getRightSpeed() - dv / 2);
	}
	
	private double driveRotationsToInches(double rotations) {
		return rotations * Constants.kDriveDiameterInches * Math.PI;
	}
	
	private double driveInchesToRotations(double in) {
		return in / (Constants.kDriveDiameterInches * Math.PI);
	}
	
	private double driveRPMToInchesPerSecond(double rpm) {
		return driveRotationsToInches(rpm)/60.0;
	}
	
	private double driveInchesPerSecondToRPM(double inps) {
		return driveInchesToRotations(inps)*60.0;
	}
	
	public double getRightDistanceInches() {
		return driveRotationsToInches(mRightDriveMaster.getPosition());
	}
	
	public double getLeftDistanceInches() {
		return driveRotationsToInches(mLeftDriveMaster.getPosition());
	}
	
	public double getRightVelocityInchesPerSec() {
		return driveRPMToInchesPerSecond(mRightDriveMaster.getSpeed());
	}
	
	public double getLeftVelocityInchesPerSec() {
		return driveRPMToInchesPerSecond(mLeftDriveMaster.getSpeed());
	}
	
	public Rotation2d getGyroAngle() {
		return Rotation2d.fromDegrees(mGyro.getAngle());
	}
	
	/**
	 * VelocityHeadingSetpoints are used to calculate the robot's path given the
	 * speed of the robot in each wheel and the polar coordinates. Especially
	 * useful if the robot is negotiating a turn and to forecast the robot's
	 * location.
	 */
	public static class VelocityHeadingSetpoint {
	    private final double mLeftSpeed;
	    private final double mRightSpeed;
	    private final Rotation2d mHeadingSetpoint;

	    // Constructor for straight line motion
	    public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
	    	mLeftSpeed = leftSpeed;
	        mRightSpeed = rightSpeed;
	        mHeadingSetpoint = headingSetpoint;
	    }

	    public double getLeftSpeed() {
	        return mLeftSpeed;
	    }

	    public double getRightSpeed() {
	        return mRightSpeed;
	    }

	    public Rotation2d getHeading() {
	        return mHeadingSetpoint;
	    }
	}
}