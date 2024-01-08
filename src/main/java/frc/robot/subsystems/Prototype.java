package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Prototype extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2;
  private MotorOutputConfigs mOutputConfigs;
  private ShuffleboardTab tab;
  private GenericEntry voltage1;
  private GenericEntry voltage2;


  public Prototype () {
    motor1 = new TalonFX(30);
    motor2 = new TalonFX(31);
    mOutputConfigs = new MotorOutputConfigs();

    
    tab = Shuffleboard.getTab("Prototype");
    voltage1 = tab.add("Motor 1 Voltage", 6)
            .withPosition(0, 0).withSize(1, 1).getEntry();
    voltage2 = tab.add("Motor 2 Voltage", 6)
            .withPosition(3, 0).withSize(1, 1).getEntry();

    configMotor(motor1);
    configMotor(motor2);
  }

  private void configMotor(TalonFX talon) {
    mOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    talon.getConfigurator().apply(mOutputConfigs);
  }

  public void run() {
    motor1.setVoltage(voltage1.getDouble(6));
    motor2.setVoltage(voltage2.getDouble(6));
  }

  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }
}
