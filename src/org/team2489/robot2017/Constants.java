package org.team2489.robot2017;

public class Constants {
	// CAN Device Chain numbers:
		public static final int kLeftDriveMaster  = 0;
		public static final int kLeftDriveSlave   = 1;

		public static final int kRightDriveMaster = 2;
		public static final int kRightDriveSlave  = 3;
		
		public static final int kShooterFlywheelMaster = 4;
		public static final int kShooterFlywheelSlave = 5;
		
		public static final int kHorizontalConveyor = 6;
		public static final int kVerticalConveyor = 7;
		
		// Driver station ports
		public static final int kLeftJoystickPort = 0;
		public static final int kRightJoystickPort = 1;
		
		// Solenoids
		public static final int kShifterA = 0;
		public static final int kShifterB = 1;
		public static final int kIntakeA = 2;
		public static final int kIntakeB = 3;

		// PID values
		public static final double kDriveVelocityKp = 2.8;
		public static final double kDriveVelocityKi = 0.0001;
		public static final double kDriveVelocityKd = 47.0;
		public static final double kDriveVelocityKf = 0.9;
		public static final int kDriveVelocityIZone = 0;
		public static final double kDriveVelocityRampRate = 0.0;
		public static final int kDriveVelocityAllowedError = 0;
		public static final int kDriveVelocitySlot = 0;
		
		public static final double kDriveStayKp = 2.8;
		public static final double kDriveStayKi = 0.0001;
		public static final double kDriveStayKd = 47.0;
		public static final double kDriveStayKf = 0.0;
		public static final int kDriveStayIZone = 0;
		public static final double kDriveStayRampRate = 0.0;
		public static final int kDriveStayAllowedError = 0;
		public static final int kDriveStaySlot = 1;
		
		public static final double kVelocityHeadingKp = 2.8;
		public static final double kVelocityHeadingKi = 0.0001;
		public static final double kVelocityHeadingKd = 47.0;
		public static final int kVelocityHeadingAllowedError = 0;
		
		public static final double kShooterVelocityKp = 2.8;
		public static final double kShooterVelocityKi = 0.0001;
		public static final double kShooterVelocityKd = 47.0;
		public static final double kShooterVelocityKf = 0.9;
		public static final int kShooterVelocityIZone = 0;
		public static final double kShooterVelocityRampRate = 0.0;
		public static final double kShooterVelocityAllowedError = 0.0;
		public static final double kShooterAccurateShotTolerance = 1.0;
		public static final double kShooterVoltageRampRate = 12.0;
		
		public static final double kVerticalConveyorCurrentKp = 2.8;
		public static final double kVerticalConveyorCurrentKi = 0.0001;
		public static final double kVerticalConveyorCurrentKd = 47.0;
		public static final double kVerticalConveyorCurrentKf = 0.9;
		public static final int kVerticalConveyorCurrentIZone = 0;
		public static final double kVerticalConveyorCurrentAllowedError = 0.0;
		public static final double kVerticalConveyorCurrentRampRate = 0.0;
		public static final double kVerticalConveyorVoltageRampRate = 12.0;

		public static final double kHorizontalConveyorCurrentKp = 2.8;
		public static final double kHorizontalConveyorCurrentKi = 0.0001;
		public static final double kHorizontalConveyorCurrentKd = 47.0;
		public static final double kHorizontalConveyorCurrentKf = 0.9;
		public static final int kHorizontalConveyorCurrentIZone = 0;
		public static final double kHorizontalConveyorCurrentAllowedError = 0.0;
		public static final double kHorizontalConveyorCurrentRampRate = 0.0;
		public static final double kHorizontalConveyorVoltageRampRate = 12.0;
		
		public static final double kConveyorTolerance = 3;

		
		// Other constants
		public static final double kMaxVelocity = 750.0;
		public static final double kMaxAccel = 1.0;
		public static final double kPathLookahead = 1.0;
		public static final double kLooperDt = 1.0;
		public static final double kPathTolerance = 0.25;
		
		
		
	    public static double kDriveWheelDiameterInches = 4.0;
	    public static double kDriveDiameterInches = 25; // NEEDS TO BE MEASURED
}
