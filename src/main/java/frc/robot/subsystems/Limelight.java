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

	private String limelightName = LimelightConstants.kLimelightName;
	private double kCameraHeight = LimelightConstants.kBackLimelightPose.getY();
	private double kMountingAngle = Units.radiansToDegrees(LimelightConstants.kBackLimelightPose.getRotation().getY());
	private double GoalHeight = LimelightConstants.kSpeakerGoalHeight;

	public Limelight() {}

	public double getDistanceToGoalInches() {
		return (GoalHeight - kCameraHeight) / Math.tan(Units.degreesToRadians(kMountingAngle + getYAngleOffsetDegrees()));
	}

	public void setGoalHeight(double GoalHeight) {
		this.GoalHeight = GoalHeight;
	}

	public double getGoalHeight() {
		return GoalHeight;
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

	public void setSpeakerPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 7);
	}

  public void setStagePipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
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
	}
}