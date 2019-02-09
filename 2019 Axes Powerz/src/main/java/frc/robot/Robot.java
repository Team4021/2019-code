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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;

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
  boolean rotationButtonLow = false;
  boolean rotationButtonMid = false;
  boolean rotationButtonTop = false;
  boolean panelPickupButton = false;
  boolean forwardTop = false;
  boolean forwardMid = false;
  boolean forwardLow = false;
  boolean forwardPickup = false;
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
  Encoder encoder1;
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    //Time = 0;
    SmartDashboard.putData("Auto choices", m_chooser);
    rearleft = new Spark(3);
    rearright = new Spark(0);
    frontleft = new Spark(2);
    frontright = new Spark(1);
    liftMotor = new Spark(4);
    Xbox = new XboxController(0);
    letsRoll = new MecanumDrive(frontleft, rearleft, frontright, rearright);
    rearright.setSafetyEnabled(false);
    rearleft.setSafetyEnabled(false);
    frontright.setSafetyEnabled(false);
    frontleft.setSafetyEnabled(false);
    liftMotor.setSafetyEnabled(false);
    encoder1 = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
    encoder1.setMaxPeriod(.1);
    encoder1.setMinRate(.01);
    encoder1.setDistancePerPulse(.045);
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
    System.out.println(distance);
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
    if (Xbox.getY(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) < -.4 || Xbox.getY(Hand.kLeft) < -.4) {
      rotationButtonLow = false;
      rotationButtonMid = false;
      rotationButtonTop = false;
      panelPickupButton = false;
      forwardTop = false;
      forwardMid = false;
      forwardLow = false;
      forwardPickup = false;
    } 
  
     if (rotationButtonTop == true || rotationButtonMid == true || rotationButtonLow == true || panelPickupButton == true) {
      autoCorrect(targetRotation, x, area);
    }
     else if (forwardTop == true) {
      placeTop();
    }
     else if (forwardMid == true) {
      placeMid();
    }  
    else if (forwardLow == true) {
      placeLow();
    }  
    else if (forwardPickup == true) {
      pickup();
    }
    else {
      letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0); // gives us control
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
      } else if (x > 1) {
        letsRoll.driveCartesian(.36, 0.0, 0, 0.0);
      } else if (area < 1.5 && area > 0) {
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


  private void placeTop() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardTop = false;
      letsRoll.driveCartesian(0, 0, 0);
    }
  }

  private void placeMid() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardMid = false;
      letsRoll.driveCartesian(0, 0, 0);
    }

  }

  private void placeLow() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardLow = false;
      letsRoll.driveCartesian(0, 0, 0);
    }

  }
  private void pickup() {
    if (distance < 25) {
      letsRoll.driveCartesian(0, .25, 0);
    } else {
      forwardPickup = false;
      letsRoll.driveCartesian(0, 0, 0);
    }
  }
}
