package org.team2489.robot2017;

import org.team2489.robot2017.utils.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired
 * range, the turret angle, and the computer vision's track's ID.
 */
public class ShooterAimingParameters {
    double range;
    Rotation2d robot_angle;
    int track_id;

    public ShooterAimingParameters(double range, Rotation2d robot_angle, int track_id) {
        this.range = range;
        this.robot_angle = robot_angle;
        this.track_id = track_id;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return robot_angle;
    }

    public int getTrackid() {
        return track_id;
    }
}
