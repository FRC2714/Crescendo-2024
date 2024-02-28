package frc.robot.utils;

public class FieldRelativeAcceleration {
    public double ax;
    public double ay;
    public double alpha;

    public FieldRelativeAcceleration(double ax, double ay, double alpha) {
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    public FieldRelativeAcceleration(FieldRelativeVelocity newSpeed, FieldRelativeVelocity oldSpeed, double time) {
        this.ax = (newSpeed.vx - oldSpeed.vx) / time;
        this.ay = (newSpeed.vy - oldSpeed.vy) / time;
        this.alpha = (newSpeed.omega - oldSpeed.omega) / time;

        if (Math.abs(this.ax) > 6.0) {
            this.ax = 6.0 * Math.signum(this.ax);
        }
        if (Math.abs(this.ay) > 6.0) {
            this.ay = 6.0 * Math.signum(this.ay);
        }
        if (Math.abs(this.alpha) > 4 * Math.PI) {
            this.alpha = 4 * Math.PI * Math.signum(this.alpha);
        }
    }

    public FieldRelativeAcceleration() {
        this.ax = 0.0;
        this.ay = 0.0;
        this.alpha = 0.0;
    }

}