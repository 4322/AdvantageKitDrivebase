package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.AHRS;
import frc.utility.OrangeMath;

public class GyroIONavX implements GyroIO {
    private AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 66);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.calibrating = gyro.isCalibrating();
        
        //log raw values which are bounded from -180 to 180
        inputs.rollPositionDeg = gyro.getRoll(); // rotation around x axis (WPI axis convenetion)
        inputs.pitchPositionDeg = gyro.getPitch(); // rotation around y axis (WPI axis convenetion)
        inputs.yawPositionDeg = gyro.getYaw(); // rotation around z axis (WPI axis convenetion)

        // yaw values used in Drive.java
        inputs.yawAngleDeg = -gyro.getAngle();
        inputs.yawVelocityDegPerSec = -gyro.getRate();
        inputs.gyroYawRotation = gyro.getRotation2d();
    }

    public void setAngleAdjustment(double adjustment) {
        gyro.setAngleAdjustment(adjustment);
    }

}
