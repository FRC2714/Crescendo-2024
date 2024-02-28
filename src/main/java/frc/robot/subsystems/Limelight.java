package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;


public class Limelight extends SubsystemBase {

	private String limelightName = LimelightConstants.kLimelightName;
	private double kCameraHeight = LimelightConstants.kBackLimelightPose.getZ();
	private double kMountingAngle = Units.radiansToDegrees(LimelightConstants.kBackLimelightPose.getRotation().getY());
	private double GoalHeight = LimelightConstants.kNoteGoalHeight;

	public Limelight() {
		setNoteSeekerPipeline();
	}

	public double getDistanceToGoalInches() {
		return (GoalHeight - kCameraHeight) / Math.tan(Units.degreesToRadians(kMountingAngle + getYAngleOffsetDegrees()));
	}

	public double getDistanceToGoalInches(double yOffsetDegrees) {
		return (GoalHeight - kCameraHeight) / Math.tan(Units.degreesToRadians(kMountingAngle + yOffsetDegrees));
	}

	public void setGoalHeight(double GoalHeight) {
		this.GoalHeight = GoalHeight;
	}

	public double getDistanceToGoalMeters() {
		return Units.inchesToMeters(getDistanceToGoalInches());
	}
	
	public double getDistanceToGoalMeters(double yOffsetDegrees) {
		return Units.inchesToMeters(getDistanceToGoalInches(yOffsetDegrees));
	}


	public double getYAngleOffsetDegrees() {
		return LimelightHelpers.getTY(limelightName);
	}

	public double getXAngleOffsetDegrees() {
		return LimelightHelpers.getTX(limelightName);
	}

	public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

	public double getYOffsetRadians() {
		return Units.degreesToRadians(getYAngleOffsetDegrees());
	}

	public Pose2d getBotPose2d() {
		return LimelightHelpers.getBotPose2d(limelightName);
	}

	public double getTimestampSeconds() {
		return Timer.getFPGATimestamp()
		- (LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0)
		- (LimelightHelpers.getLatency_Capture(limelightName) / 1000.0);
	}

	public double getPrimaryAprilTagId() {
		return LimelightHelpers.getFiducialID(limelightName);
	}

	public boolean isTargetVisible() {
		return LimelightHelpers.getTV(limelightName);
	}

	public void setLED(boolean lightOn) {
        if (lightOn) LimelightHelpers.setLEDMode_ForceOn(limelightName); // LED force on
        else LimelightHelpers.setLEDMode_ForceOff(limelightName); // LED force off
    }

	public void setAprilTagPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}

	public void setSpeakerPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 4); // tune later
	}

	public void setNoteSeekerPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 0); // tune later
	}

  	public void setStagePipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}

	public void setPosePipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 5); // TBD
	}

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("ll distance to goal", getDistanceToGoalMeters());
		SmartDashboard.putNumber("ll rotation from apriltag", isTargetVisible() ? getXAngleOffsetDegrees() : 30);
		SmartDashboard.putBoolean("ll is target visible", isTargetVisible());
	}
}