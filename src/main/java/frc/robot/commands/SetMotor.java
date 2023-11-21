package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Motor;

public class SetMotor extends CommandBase {
  private final Motor motor;

  public SetMotor(Motor motor) {
    this.motor = motor;
  }

  @Override
  public void initialize() {
    motor.setMotorSpeed(-0.2);
    Logger.getInstance().recordOutput("B-Button", 1);
  }

  @Override
  public void end(boolean interrupted) {
    motor.setMotorSpeed(0);
    Logger.getInstance().recordOutput("B-Button", 1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}