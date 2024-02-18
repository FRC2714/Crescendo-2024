package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRelativeVelocity {
    public double vx;
    public double vy;
    public double omega;

    public FieldRelativeVelocity(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public FieldRelativeVelocity(ChassisSpeeds chassisSpeed, Rotation2d gyro) {
        this(chassisSpeed.vxMetersPerSecond * gyro.getCos() - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
                chassisSpeed.vyMetersPerSecond * gyro.getCos() + chassisSpeed.vxMetersPerSecond * gyro.getSin(),
                chassisSpeed.omegaRadiansPerSecond);
    }

    public FieldRelativeVelocity() {
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
    }

}