package org.team2489.robot2017.loops;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionProcessor implements Loop {
    private static final double kEpsilon = 1E-9;
    
	static VisionProcessor instance = new VisionProcessor();
	
	NetworkTable mVisionTable;
	double last_sample_id;
	
	public static VisionProcessor getInstance() {
		return instance;
	}
	
	public VisionProcessor() {
	}
	
	@Override
	public void onStart() {
		mVisionTable = NetworkTable.getTable("vision_table");
		last_sample_id = -1;
	}
	
	@Override
	public void onLoop() {
		double sample_id = mVisionTable.getNumber("last_sample_id", -1);
		if(Math.abs(last_sample_id - sample_id) < kEpsilon) {
			last_sample_id = sample_id;
			double x = mVisionTable.getNumber("vision_target_x", -1E5);
			double y = mVisionTable.getNumber("vision_target_y", -1E5);
			double z = mVisionTable.getNumber("vision_target_z", -1E5);
			double timestamp = mVisionTable.getNumber("image_timestamp", -1);
			
			if (x < -1E4 || y < -1E4 || z < -1E4 || timestamp < 0) {
				System.out.println("Target not found");
			}
		}
	}
	
	@Override
	public void onStop() {
		last_sample_id = -1;
	}
}
