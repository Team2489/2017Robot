package org.team2489.robot2017.auto.actions;
import org.team2489.robot2017.subsystems.Drivetrain;
import org.team2489.robot2017.utils.Path;

public class FollowPathAction implements Action {
    private Drivetrain mDrive = Drivetrain.getInstance();

    private Path mPath;
    private boolean mReversed;
    private boolean mHasStarted;

    public FollowPathAction(Path path, boolean reversed) {
        mPath = path;
        mReversed = reversed;
        mHasStarted = false;
    }

    @Override
    public boolean isFinished() {
        boolean done = mDrive.isFinishedPath() && mHasStarted;
        if (done) {
            System.out.println("Finished path");
        }
        return done;
    }

    @Override
    public void update() {
        mHasStarted = true;
    }

    @Override
    public void done() {
        mDrive.stop();
    }

    @Override
    public void start() {
        mDrive.followPath(mPath, mReversed);
    }
}
