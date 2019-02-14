/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // The following line is getting variables from the limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // tx and ty are angles from the crosshairs on the object to origin
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  // area of the object
  NetworkTableEntry ta = table.getEntry("ta");
  // next 4 are lengths of longest and shortest sides, and horizontal and vertical
  // distances
  NetworkTableEntry tlong = table.getEntry("tlong");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry thor = table.getEntry("thor");
  // this tells us what "pipeline" we are on, basically different settings for the
  // camera
  NetworkTableEntry getpipe = table.getEntry("getpipe");
  // skew or rotation of target
  NetworkTableEntry ts = table.getEntry("ts");
  private MecanumDrive letsRoll;
  private XboxController Xbox;
  double distance;
  double ledMode = 0;
  double PSI;
  boolean rotationButtonLow = false;
  boolean rotationButtonMid = false;
  boolean rotationButtonTop = false;
  boolean panelPickupButton = false;
  boolean retreatVariable;
  boolean forwardTop = false;
  boolean forwardMid = false;
  boolean forwardLow = false;
  boolean forwardPickup = false;
  boolean clawOpen = true;
  boolean strafeButton;
  boolean panelVariable;
  boolean switchLowValue;
  boolean switchMidValue;
  boolean switchTopValue;
  float Kp;
  Spark rearleft;
  Spark rearright;
  Spark frontleft;
  Spark frontright;
  Spark liftMotor;
  Relay Spike;
  Encoder encoder1;
  Compressor compressor = new Compressor(0);
  Solenoid solenoid = new Solenoid(0);
  AnalogInput pressureSensor = new AnalogInput(0);
  DigitalInput limitLow = new DigitalInput(0);
  DigitalInput limitMid = new DigitalInput(1);
  DigitalInput limitTop = new DigitalInput(2);
  DigitalInput limitFront = new DigitalInput(3);
  DigitalInput limitBack = new DigitalInput(4);
  DigitalInput limitRightWheel = new DigitalInput(5);
  DigitalInput limitLeftWheel = new DigitalInput(6);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putData("Auto choices", m_chooser);
    rearleft = new Spark(3);
    rearright = new Spark(0);
    frontleft = new Spark(2);
    frontright = new Spark(1);
    liftMotor = new Spark(4);
    Spike = new Relay(0);
    Xbox = new XboxController(0);
    letsRoll = new MecanumDrive(frontleft, rearleft, frontright, rearright);
    // Safety errors if the following are true
    rearright.setSafetyEnabled(false);
    rearleft.setSafetyEnabled(false);
    frontright.setSafetyEnabled(false);
    frontleft.setSafetyEnabled(false);
    liftMotor.setSafetyEnabled(false);
    encoder1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    encoder1.setMaxPeriod(.1);
    encoder1.setMinRate(.01);
    // encoder1.setDistancePerPulse(.045);
    encoder1.setDistancePerPulse(.062);
    encoder1.setReverseDirection(false);
    encoder1.setSamplesToAverage(7);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    distance = encoder1.getDistance();
    // calculating, printing, and putting PSI to SmartDashboard
    double PSI = 250 * pressureSensor.getVoltage() / 5.0 - 20.0;
    System.out.println(PSI);
    SmartDashboard.putNumber("PSI", PSI);
    // these put our NetworkTableEntries into variables
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double longestSide = tlong.getDouble(0.0);
    double shortestSide = tshort.getDouble(0.0);
    double targetWidth = thor.getDouble(0.0);
    double targetHeight = tvert.getDouble(0.0);
    double pipeline = getpipe.getDouble(0.0);
    double targetRotation = ts.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x); // displays x axis from target
    SmartDashboard.putNumber("LimelightY", y); // displays y axis from target
    SmartDashboard.putNumber("LimelightArea", area); // displays area of target
    SmartDashboard.putNumber("LimelightRotation", targetRotation); // displays rotation of target
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    if (PSI < 115) { // Maximum legal PSI is 120, sometimes overshoots
      compressor.setClosedLoopControl(true);
    } else {
      compressor.setClosedLoopControl(false);
    }
    if (clawOpen == true) {
      solenoid.set(true);
      // might have to write close code?
    }

    if (Xbox.getXButtonPressed()) {
      rotationButtonTop = true;
    } // starts panel place on top
    if (Xbox.getYButtonPressed()) {
      rotationButtonMid = true;
    } // starts panel place on Mid
    if (Xbox.getBButtonPressed()) {
      rotationButtonLow = true;
    } // starts panel place on Low
    if (Xbox.getAButtonPressed()) {
      panelPickupButton = true;
    } // starts panel pickup
    if (Xbox.getY(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) < -.4
        || Xbox.getY(Hand.kLeft) < -.4) {
      // Jumps out of semiautonomous if joystick is moved far enough
      rotationButtonLow = false;
      rotationButtonMid = false;
      rotationButtonTop = false;
      panelPickupButton = false;
      forwardTop = false;
      forwardMid = false;
      forwardLow = false;
      forwardPickup = false;
      retreatVariable = false;
    }

    if (rotationButtonTop == true || rotationButtonMid == true || rotationButtonLow == true
        || panelPickupButton == true) {
      // If any buttons are true, go to autoCorrect
      // They go below because they are different variables
      autoCorrect(targetRotation, x, area);
    } else if (forwardTop == true) {
      placeTop();
      // In these and the following, we go to the desired method
    } else if (forwardMid == true) {
      placeMid();
    } else if (forwardLow == true) {
      placeLow();
    } else if (forwardPickup == true) {
      pickup();
    } else if (retreatVariable == true) {
      pinchAndRetreat();
    } else {
      letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0);
      // gives us control
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void autoCorrect(double targetRotation, double x, double area) {
    if (Math.abs(targetRotation) > 45 && Math.abs(targetRotation) < 89) {
      // arc right
      letsRoll.driveCartesian(.3, 0.0, -.125, 0.0);
    } else if (Math.abs(targetRotation) < 45 && Math.abs(targetRotation) > 1) {
      letsRoll.driveCartesian(-.3, 0.0, .125, 0.0);
      // arc left
    } else if (targetRotation < -88 || targetRotation > -2) {
      if (x < -1) {
        letsRoll.driveCartesian(-.36, 0.0, 0, 0.0);
        // If on the left side of target, go right
      } else if (x > 1) {
        letsRoll.driveCartesian(.36, 0.0, 0, 0.0);
        // If on the right side of target, go left
      } else if (area < 1.5 && area > 0) {
        // We need to change the areas above because of the camera's new postition
        letsRoll.driveCartesian(0, .5, 0, 0.0);
      } else {
        frontleft.set(0);
        frontright.set(0);
        rearleft.set(0);
        rearright.set(0);
        encoder1.reset();
        if (rotationButtonTop == true) {
          rotationButtonTop = false;
          forwardTop = true;
          // Makes sure we don't do autocorrect again by using different variables, same
          // things below
        } else if (rotationButtonMid == true) {
          rotationButtonMid = false;
          forwardMid = true;
        } else if (rotationButtonLow == true) {
          rotationButtonLow = false;
          forwardLow = true;
        } else if (panelPickupButton == true) {
          panelPickupButton = false;
          forwardPickup = true;
        }
      }
    }

  }

  private void placeTop() { // Test top first!!! Not others
    if (limitTop.get() == false) {
      liftMotor.set(.75);
      // If the top limit switch is not pressed, go up
    } else if (limitFront.get() == false) {
      Spike.set(Value.kForward);
      // If the front limit switch is not pressed, move the claw forward
    } else {
      forwardTop = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
      // Don't repeat this method, stop, prepare to pinchAndRetreat
    }
  }

  private void placeMid() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardMid = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
    }

  }

  private void placeLow() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardLow = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
    }

  }

  private void pickup() {
    if (limitLow.get() == false) {
      liftMotor.set(-.75);
      // If the lift isn't in the lowest setting (sensed by limit switch) go down
    } else if (limitFront.get() == false && clawOpen == false) {
      Spike.set(Value.kForward);
      // If the claw isn't forward and isn't open, move the claw forward (and
      // statement explained below)
    } else if (clawOpen == false) {
      clawOpen = true;
      // Makes sure the claw is open
    } else if (distance > -10) {
      letsRoll.driveCartesian(0, -.5, 0);
      // Moves backwards until the encoder is low enough
    } else if (limitBack.get() == false) {
      Spike.set(Value.kReverse);
      // If the claw isn't in the back position, move back. If we didn't have the and
      // statement, it would get stuck going back and forth between these statements
    } else {
      forwardPickup = false;
      // Don't repeat this method
    }
  }

  private void pinchAndRetreat() {
    clawOpen = false;
    if (distance > -10) {
      letsRoll.driveCartesian(0, -.5, 0);
      // Move back until the encoder gets to -10
    } else {
      retreatVariable = false;
      // Don't repeat this method
    }

  }
}
