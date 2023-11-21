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
        inputs.rollPositionDeg = gyro.getRoll();
        inputs.pitchPositionDeg = gyro.getPitch();
        inputs.yawPositionDeg = -gyro.getAngle();
        inputs.yawPositionRad = Math.toRadians(-gyro.getAngle());
        inputs.yawVelocityDegPerSec = -gyro.getRate();
    }

    public void setAngleAdjustment(double adjustment) {
        gyro.setAngleAdjustment(adjustment);
    }

}
