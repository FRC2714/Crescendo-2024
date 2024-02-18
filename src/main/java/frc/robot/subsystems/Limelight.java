package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.utils.LimelightHelpers;


public class Limelight extends SubsystemBase {

	private String limelightName = "limelight-front";
	private double kCameraHeight = 9.14;
	private double kMountingAngle = -4;
	private double GoalHeight = 0; //inches, deg NEEDS TO BE UPDATED

	public Limelight() {}

	public double getDistanceToGoalInches() {
		return (GoalHeight - kCameraHeight) / Math.tan(Units.degreesToRadians(kMountingAngle + getYAngleOffsetDegrees()));
	}

	public void setGoalHeight(double GoalHeight) {
		this.GoalHeight = GoalHeight;
	}

	public double getDistanceToGoalMeters() {
		return Units.inchesToMeters(getDistanceToGoalInches());
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

	public void setNoteSeekerPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 3); // tune later
	}

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	public Pose2d getBotPose2d() {
		return LimelightHelpers.getBotPose2d(limelightName);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("ll distance to goal", getDistanceToGoalMeters());
		SmartDashboard.putNumber("ll rotation from apriltag", isTargetVisible() ? getXAngleOffsetDegrees() : 30);
		SmartDashboard.putBoolean("ll is target visibe", isTargetVisible());
	}
}