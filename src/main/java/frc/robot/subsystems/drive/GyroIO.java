package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public boolean calibrating = false;
        public double rollPositionDeg = 0.0;
        public double pitchPositionDeg = 0.0;
        public double yawPositionDeg = 0.0;
        public double yawPositionRad = 0.0;
        public double yawVelocityDegPerSec = 0.0;
        public Rotation2d gyroYawRotation = new Rotation2d();
        public double yawAngleDeg = 0.0;

    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setAngleAdjustment(double adjustment) {}

}