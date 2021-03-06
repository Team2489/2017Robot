package org.team2489.robot2017;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
//import java.util.List;
import java.util.Map;

import org.team2489.robot2017.utils.*;
import org.team2489.robot2017.GoalTracker;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a point and direction in space that
 * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
 * spatial relationship between different frames.
 * 
 * Robot frames of interest (from parent to child):
 * 
 * 1. Field frame: origin is where the robot is turned on
 * 
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 * 
 * 3. Turret fixed frame: origin is the center of the turret when the turret is
 * at 0 degrees rotation relative to the vehicle frame
 * 
 * 4. Turret rotating frame: origin is the center of the turret as it rotates
 * 
 * 5. Camera frame: origin is the center of the camera imager as it rotates with
 * the turret
 * 
 * 6. Goal frame: origin is the center of the goal (note that orientation in
 * this frame is arbitrary). Also note that there can be multiple goal frames.
 * 
 * As a kinematic chain with 6 frames, there are 5 transforms of interest:
 * 
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 * 
 * 2. Vehicle-to-turret-fixed: This is a constant.
 * 
 * 3. Vehicle-to-turret-rotating: This is a pure rotation, and is tracked over
 * time using the turret encoder.
 * 
 * 4. Turret-rotating-to-camera: This is a constant.
 * 
 * 5. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 */

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;

//    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
//            new Translation2d(Constants.kTurretXOffset, Constants.kTurretYOffset),
//            Rotation2d.fromDegrees(Constants.kTurretAngleOffsetDegrees));
    public static final RigidTransform2d kVehicleToCameraFixed = new RigidTransform2d(
    		new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset),
    		Rotation2d.fromDegrees(0));
    

//    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
//            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> field_to_vehicle_;
    protected RigidTransform2d.Delta vehicle_velocity_;
    protected GoalTracker mGoalTracker;
    protected Rotation2d camera_pitch_correction_;
    protected Rotation2d camera_yaw_correction_;
    protected double differential_height_;

    protected RobotState() {
        reset(0, new RigidTransform2d(), new Rotation2d());
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle,
            Rotation2d initial_turret_rotation) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_ = new RigidTransform2d.Delta(0, 0, 0);
        mGoalTracker = new GoalTracker();
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        differential_height_ = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
    }

    public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized RigidTransform2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue().transformBy(
                RigidTransform2d.fromVelocity(new RigidTransform2d.Delta(vehicle_velocity_.dx * lookahead_time,
                        vehicle_velocity_.dy * lookahead_time, vehicle_velocity_.dtheta * lookahead_time)));
    }

    public synchronized RigidTransform2d getFieldToCamera(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCameraFixed);
    }

    public synchronized RigidTransform2d getCaptureTimeFieldToGoal() {
    	RigidTransform2d rt = null;
    	GoalTracker.TrackReport report = mGoalTracker.getTrack();
        rt = RigidTransform2d.fromTranslation(report.field_to_goal);
        
        return rt;
    }

    public synchronized ShooterAimingParameters getAimingParameters(double current_timestamp) {
        ShooterAimingParameters sap;
        GoalTracker.TrackReport report = mGoalTracker.getTrack();

        // turret fixed (latest) -> vehicle (latest) -> field
        RigidTransform2d latest_vehicle_to_field = getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime)
        		.inverse();

        if (report == null) {
        	return null;
        }
        
        if (current_timestamp - report.latest_timestamp > kMaxTargetAge) {
            return null;
        }
        // turret fixed (latest) -> vehicle (latest) -> field -> goals
        RigidTransform2d latest_vehicle_to_goal = latest_vehicle_to_field
                .transformBy(RigidTransform2d.fromTranslation(report.field_to_goal));

        // We can actually disregard the angular portion of this pose. It is
        // the bearing that we care about!
        sap = new ShooterAimingParameters(latest_vehicle_to_goal.getTranslation().norm(),
                new Rotation2d(latest_vehicle_to_goal.getTranslation().getX(),
                        latest_vehicle_to_goal.getTranslation().getY(), true),
                report.id);
        
        return sap;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d field_to_vehicle
    		, RigidTransform2d.Delta velocity) {
        addFieldToVehicleObservation(timestamp, field_to_vehicle);
        vehicle_velocity_ = velocity;
    }

    public void addVisionUpdate(double timestamp, TargetInfo target) {
        Translation2d field_to_goal = null;
        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        
        if (target != null) {
            double y = target.getY();

            // Compensate for camera yaw
            double xyaw = target.getX() * camera_yaw_correction_.cos() + y * camera_yaw_correction_.sin();
            double yyaw = y * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
            double zyaw = target.getZ();

            // Compensate for camera pitch
            double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
            double yr = yyaw;
            double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();

            // find intersection with the goal
            if (zr > 0) {
                double scaling = differential_height_ / zr;
                double distance = Math.hypot(xr, yr) * scaling;
                Rotation2d angle = new Rotation2d(xr, yr, true);
                field_to_goal = field_to_camera
                        .transformBy(RigidTransform2d
                                .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                        .getTranslation();
            }
        }
        synchronized (this) {
        	if (field_to_goal != null) {
        		mGoalTracker.update(timestamp, field_to_goal);
        	}
        }
    }

    public synchronized void resetVision() {
        mGoalTracker.reset();
    }

    public RigidTransform2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
        return Kinematics.integrateForwardKinematics(last_measurement, left_encoder_delta_distance,
                right_encoder_delta_distance, current_gyro_angle);
    }

    public void outputToSmartDashboard() {
        RigidTransform2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().getX());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        RigidTransform2d pose = getCaptureTimeFieldToGoal();
        
        if (pose != null) {
        	SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().getX());
        	SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().getY());
        }
    }
}