package org.team2489.robot2017.loops;
import edu.wpi.first.wpilibj.Timer;
import org.team2489.robot2017.subsystems.Drivetrain;
import org.team2489.robot2017.RobotState;
import org.team2489.robot2017.Kinematics;
import org.team2489.robot2017.utils.RigidTransform2d;
import org.team2489.robot2017.utils.Rotation2d;

public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState mRobotState = RobotState.getInstance();
    Drivetrain mDrive = Drivetrain.getInstance();

    double mLeftEncoderPrevDistance = 0;
    double mRightEncoderPrevDistance = 0;

    @Override
    public void onStart() {
    	mLeftEncoderPrevDistance = mDrive.getLeftDistanceInches();
    	mRightEncoderPrevDistance = mDrive.getRightDistanceInches();
    }

    @Override
    public void onLoop() {
        double time = Timer.getFPGATimestamp();
        double left_distance = mDrive.getLeftDistanceInches();
        double right_distance = mDrive.getRightDistanceInches();
        Rotation2d gyro_angle = mDrive.getGyroAngle();
        
        RigidTransform2d odometry = mRobotState.generateOdometryFromSensors(
                left_distance - mLeftEncoderPrevDistance, right_distance - mRightEncoderPrevDistance, gyro_angle);
        RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(mDrive.getLeftVelocityInchesPerSec(),
                mDrive.getRightVelocityInchesPerSec());
        
        mRobotState.addObservations(time, odometry, velocity);
        mLeftEncoderPrevDistance = left_distance;
        mRightEncoderPrevDistance = right_distance;
    }

    @Override
    public void onStop() {
        // no-op
    }

}