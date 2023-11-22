// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.WheelPosition;
import frc.robot.Constants.DriveConstants.Auto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController xbox = new CommandXboxController(2);
  public static Joystick driveStick;
  public static Joystick rotateStick;

  private JoystickButton driveTrigger;
  private JoystickButton driveButtonSeven;
  private JoystickButton driveButtonNine;
  private JoystickButton driveButtonEleven;
  private JoystickButton driveButtonTwelve;
  private JoystickButton driveButtonThree;
  private JoystickButton rotateButtonFour;

  private JoystickButton rotateTrigger;

  private ShuffleboardTab tab;

  private final Drive drive = new Drive(new GyroIONavX(), 
                              new SwerveModuleIOTalonFX(WheelPosition.FRONT_RIGHT), new SwerveModuleIOSparkMax(WheelPosition.FRONT_RIGHT),
                              new SwerveModuleIOTalonFX(WheelPosition.FRONT_LEFT), new SwerveModuleIOSparkMax(WheelPosition.FRONT_LEFT),
                              new SwerveModuleIOTalonFX(WheelPosition.BACK_RIGHT), new SwerveModuleIOSparkMax(WheelPosition.BACK_RIGHT),
                              new SwerveModuleIOTalonFX(WheelPosition.BACK_LEFT), new SwerveModuleIOSparkMax(WheelPosition.BACK_LEFT));

  private final DriveManual driveManualDefault = new DriveManual(drive, DriveManual.AutoPose.none);
  private final DriveStop driveStop = new DriveStop(drive);

  private int selectedPosition = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

      // Replayed robot, disable IO implementations
      default:
        break;
    }

    drive.init();

    configureButtonBindings();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    /* 
    if (subsystem == null) {
      myMotor = new Subsystem(new SubsystemIO() {});
    }
    */

    // Set up auto routines

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);

      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonTwelve = new JoystickButton(driveStick, 12);

      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonTwelve.onTrue(driveStop);
    }

    if (Constants.xboxEnabled) {
      xbox.povUp().onTrue(new ResetFieldCentric(drive, 0, true));
    }
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      disableTimer.stop();
      disableTimer.reset();
    }
  }

  public void enableSubsystems() {
    drive.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    driveStop.schedule();  // interrupt all drive commands
    disableTimer.reset();
    disableTimer.start();
  }

  public Command getAutonomousCommand() {}





}
