package org.team2489.robot2017;

import org.team2489.robot2017.utils.Translation2d;

public class GoalTracker {
    /**
     * Track reports contain all of the relevant information about a given goal
     * track.
     */
    public static class TrackReport {
        // Translation from the field frame to the goal
        public Translation2d field_to_goal;

        // The timestamp of the latest time that the goal has been observed
        public double latest_timestamp;

        // The track id
        public int id;

        public TrackReport(GoalTrack track) {
            this.field_to_goal = track.getSmoothedPosition();
            this.latest_timestamp = track.getLatestTimestamp();
            this.id = track.getId();
        }
    }


    GoalTrack mCurrentTrack;
    int mNextId = 0;

    public GoalTracker() {
    }

    public void reset() {
        mCurrentTrack = null;
    }

    public void update(double timestamp, Translation2d field_to_goal) {
        // Try to update existing track
        if (mCurrentTrack != null) {
        	if (!(mCurrentTrack.tryUpdate(timestamp, field_to_goal))) {
        		mCurrentTrack.emptyUpdate();
        	}
        }
        
        // Prune track if it has died
        if (!mCurrentTrack.isAlive()) {
        	mCurrentTrack = null;
        }

        
        // If all tracks are dead, start new tracks for any detections
        if (mCurrentTrack == null) {
        	mCurrentTrack = (GoalTrack.makeNewTrack(timestamp, field_to_goal, mNextId));
            ++mNextId;
        }
    }

    public boolean hasTrack() {
        return mCurrentTrack == null;
    }

    public TrackReport getTrack() {
    	if (hasTrack()) {
    		TrackReport tr = new TrackReport(mCurrentTrack);
    		return tr;
        }
        return null;
    }
}
