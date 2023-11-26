import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveManualLogic;
import frc.utility.OrangeMath;

// unit test naming convention: test<MethodToTest>_desiredState

public class DriveManualLogicTest {
  
  @BeforeEach
  public void setup() {
    return;
  }

  @AfterEach
  public void shutdown() {
    return;
  }

  @Test
  public void testDriveNormalizationScalingBadInputs_45Angle() {
    Translation2d translation = DriveManualLogic.doDriveNormalizationScaling(1, 1, DriveConstants.Manual.joystickDriveDeadband, 3);
    System.out.println(translation.getAngle().getDegrees());
    System.out.println(translation.getX());
    System.out.println(translation.getY());
    assertTrue(OrangeMath.equalToTwoDecimal(45, translation.getAngle().getDegrees()));
  }

  @Test
  public void testDriveNormalizationScalingGoodInputs_30Angle() {
    Translation2d translation = DriveManualLogic.doDriveNormalizationScaling(Math.sqrt(3)/2, 0.5, DriveConstants.Manual.joystickDriveDeadband, 3);
    System.out.println(translation.getAngle().getDegrees());
    System.out.println(translation.getX());
    System.out.println(translation.getY());
    assertTrue(OrangeMath.equalToTwoDecimal(30, translation.getAngle().getDegrees()));
  }

  @Test
  public void testDriveNormalizationScalingBadInputs_30Angle() {
    Translation2d translation = DriveManualLogic.doDriveNormalizationScaling(1, 1/Math.sqrt(3), DriveConstants.Manual.joystickDriveDeadband, 3);
    System.out.println(translation.getAngle().getDegrees());
    System.out.println(translation.getX());
    System.out.println(translation.getY());
    assertTrue(OrangeMath.equalToTwoDecimal(30, translation.getAngle().getDegrees()));
  }

  @Test
  public void testDriveNormalizationScalingBadInputs_OtherAngle() {
    Translation2d translation = DriveManualLogic.doDriveNormalizationScaling(0.9, 1, DriveConstants.Manual.joystickDriveDeadband, 3);
    System.out.println(translation.getAngle().getDegrees());
    System.out.println(translation.getX());
    System.out.println(translation.getY());
    assertTrue(OrangeMath.equalToTwoDecimal(48.0127, translation.getAngle().getDegrees()));
  }

}