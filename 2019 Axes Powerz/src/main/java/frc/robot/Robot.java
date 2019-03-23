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
//import java.io.FileWriter;
//import java.io.IOException;
//import java.io.PrintWriter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.VictorSP;
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
  private static final String limelightMethod = "Limelight";
  private static final String jackdrive = "Manual Override";
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
  double camx;
  double camy;
  double camarea;
  double targetRotation;
  double distance;
  double ledMode = 0;
  double cammode = 0;
  double PSI;
  boolean rotationButtonLow = false;
  boolean rotationButtonMid = false;
  boolean rotationButtonTop = false;
  boolean panelPickupButton = false;
  boolean retreatVariable = false;
  boolean forwardTop = false;
  boolean forwardMid = false;
  boolean forwardLow = false;
  boolean forwardPickup;
  boolean clawOpen = true;
  boolean strafeButton = false;
  boolean panelVariable = false;
  boolean flippyBoi = false;
  boolean runCompress = true;
  float Kp;
  VictorSP rearleft;
  VictorSP rearright;
  VictorSP frontleft;
  VictorSP frontright;
  VictorSP liftMotor;
  VictorSP liftBot;
  VictorSP liftBotSpinRight;
  VictorSP liftBotSpinLeft;
  Relay Spike;
  Encoder encoder1;
  Compressor compressor = new Compressor(0);
  Solenoid solenoid = new Solenoid(0);
  AnalogInput pressureSensor = new AnalogInput(0);
  DigitalInput limitLow;
  DigitalInput limitMid;
  DigitalInput limitTop;
  DigitalInput limitFront;
  DigitalInput limitBack;
  DigitalInput limitWheelFront;
  DigitalInput limitWheelBack;
  // FileWriter writer;
  // PrintWriter printWriter;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("limelight", limelightMethod);
    m_chooser.addOption("JackDrive", jackdrive);
    SmartDashboard.putData("drive choices", m_chooser);
    limitLow = new DigitalInput(0);
    limitMid = new DigitalInput(4);
    limitTop = new DigitalInput(6);
    limitFront = new DigitalInput(1);
    limitBack = new DigitalInput(5);
    limitWheelFront = new DigitalInput(2);
    limitWheelBack = new DigitalInput(3);
    // Ports are subject to change,
    rearleft = new VictorSP(4); // 3 on prototype
    rearright = new VictorSP(3); // 0 on prototype
    frontleft = new VictorSP(6); // 2 on prototype
    frontright = new VictorSP(2); // 1 prototype
    liftMotor = new VictorSP(0); // 4 prototype
    liftBot = new VictorSP(5);
    liftBotSpinLeft = new VictorSP(1);
    liftBotSpinRight = new VictorSP(7);
    Spike = new Relay(0);
    Xbox = new XboxController(0);
    letsRoll = new MecanumDrive(frontleft, rearleft, frontright, rearright);
    // Safety errors if the following are true
    rearright.setSafetyEnabled(false);
    rearleft.setSafetyEnabled(false);
    frontright.setSafetyEnabled(false);
    frontleft.setSafetyEnabled(false);
    liftMotor.setSafetyEnabled(false);
    encoder1 = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
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
  public void robotPeriodic() 
  {
    if (Xbox.getRawButton(7)) 
    {
      rotationButtonLow = false;
      rotationButtonMid = false;
      rotationButtonTop = false;
      panelPickupButton = false;
      forwardTop = false;
      forwardMid = false;
      forwardLow = false;
      forwardPickup = false;
      retreatVariable = false;
      flippyBoi = false;
    }
    if (isDisabled()) 
    {
      rotationButtonLow = false;
      rotationButtonMid = false;
      rotationButtonTop = false;
      panelPickupButton = false;
      forwardTop = false;
      forwardMid = false;
      forwardLow = false;
      forwardPickup = false;
      retreatVariable = false;
      flippyBoi = false;
    }

    compressor.setClosedLoopControl(true);
    m_autoSelected = m_chooser.getSelected();
    SmartDashboard.putBoolean("LimitTop", limitTop.get());
    SmartDashboard.putBoolean("LimitMid", limitMid.get());
    SmartDashboard.putBoolean("LimitLow", limitLow.get());
    SmartDashboard.putBoolean("LimitFront", limitFront.get());
    SmartDashboard.putBoolean("LimitBack", limitBack.get());
    SmartDashboard.putBoolean("LimitWheelFront", limitWheelFront.get());
    SmartDashboard.putBoolean("LimitWheelback", limitWheelBack.get());
    SmartDashboard.putNumber("Encoder", distance);
    distance = encoder1.getDistance();
    // calculating, printing, and putting PSI to SmartDashboard
    // these put our NetworkTableEntries into variables
    camx = tx.getDouble(0.0);
    camy = ty.getDouble(0.0);
    camarea = ta.getDouble(0.0);
    // double longestSide = tlong.getDouble(0.0);
    // double shortestSide = tshort.getDouble(0.0);
    // double targetWidth = thor.getDouble(0.0);
    // double targetHeight = tvert.getDouble(0.0);
    // double pipeline = getpipe.getDouble(0.0);
    targetRotation = ts.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", camx); // displays x axis from target
    SmartDashboard.putNumber("LimelightY", camy); // displays y axis from target
    SmartDashboard.putNumber("LimelightArea", camarea); // displays area of target
    SmartDashboard.putNumber("LimelightRotation", targetRotation); // displays rotation of target
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    if (clawOpen == true) 
    {
      solenoid.set(true);
    } else {
      solenoid.set(false);
    }
    // System.out.println("auto select is " + m_autoSelected);
    switch (m_autoSelected) 
    {
      case limelightMethod: Normal();
           break;
      
      case jackdrive:
      default:
      manualOverride();
      break;
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
    //m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
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
  public void testPeriodic() 
  {
  }

  private void autoCorrect() 
  {
    if (Xbox.getRawButton(5)) 
    {
      clawOpen = true;
    } 
    else if (Xbox.getRawButton(6)) 
    {
      clawOpen = false;
    }

    if (isEnabled())
    {
      System.out.println("autocorrect engaged" + ", targetRotation " + targetRotation + ", area " + camarea + ", x " + camx);
    }

    if (Math.abs(targetRotation) >= 75 && Math.abs(targetRotation) <= 89) 
    {
      if (isEnabled())
      {
        System.out.println("autocorrect Step 1"); 
      }
      // arc right
      letsRoll.driveCartesian(.3, 0.0, .125, 0.0);
    }
    else if (Math.abs(targetRotation) >= 1 && Math.abs(targetRotation) <= 15) 
    {
      if (isEnabled())
      {
        System.out.println("autocorrect Step 2");
      }
      letsRoll.driveCartesian(-.3, 0.0, -.125, 0.0); // arc left
    } 
    else 
    {
      if (isEnabled())
      {
        System.out.println("autocorrect Step 3");
      }
      if (camx < -1.5) 
      {
        letsRoll.driveCartesian(.36, 0.0, 0, 0.0);
        // If on the left side of target, go right
      } 
      else if (camx > 1.5) 
      {
        if (isEnabled())
        {
          System.out.println("autocorrect Step 4");
        }
        letsRoll.driveCartesian(-.36, 0.0, 0, 0.0);
        // If on the right side of target, go left
      } 
      else if (camarea > 0 && camarea < 20) 
      {
        if (isEnabled())
        {
          System.out.println("autocorrect Step 5");
        }
        // We need to change the areas above because of the camera's new postition
        letsRoll.driveCartesian(0, .5, 0, 0.0);
        encoder1.reset();
      } 
      else 
      {
        if (isEnabled())
        {
          System.out.println("autocorrect Step 6");
        }
        frontleft.set(0);
        frontright.set(0);
        rearleft.set(0);
        rearright.set(0);
        encoder1.reset();
        if (rotationButtonTop == true) 
        {
          if (isEnabled())
          {
            System.out.println("autocorrect Initialize go top");
          }
          rotationButtonTop = false;
          forwardTop = true;
          // Makes sure we don't do autocorrect again by using different variables, same
          // things below
        } 
        else if (rotationButtonMid == true) 
        {
          if (isEnabled())
          {
            System.out.println("autocorret Initialize go mid");
          }
          rotationButtonMid = false;
          forwardMid = true;
        } 
        else if (rotationButtonLow == true) 
        {
          if (isEnabled())
          {
            System.out.println("autocorrect Initialize go low");
          }
          rotationButtonLow = false;
          forwardLow = true;
        } 
        else if (panelPickupButton == true) 
        {
          if (isEnabled())
          {
            System.out.println("autocorrect Initialize Pickup");
          }
          panelPickupButton = false;
          forwardPickup = true;
          clawOpen = false;
        }
      }
    }
  }
  

  private void placeTop() 
  { // Test top first!!! Not others
    if (limitTop.get() == false) 
    {
      liftMotor.set(.75);
      // If the top limit switch is not pressed, go up
    } 
    else if (limitFront.get() == false) 
    {
      // Spike.set(Value.kForward);
      // If the front limit switch is not pressed, move the claw forward
    } 
    else 
    {
      forwardTop = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
      // Don't repeat this method, stop, prepare to pinchAndRetreat
    }
  }

  private void placeMid() 
  {
    if (limitMid.get() == false) 
    {
      liftMotor.set(.75);
      // If the top limit switch is not pressed, go up
    } 
    else if (limitFront.get() == false) 
    {
      Spike.set(Value.kForward);
      // If the front limit switch is not pressed, move the claw forward
    } 
    else 
    {
      forwardMid = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
    }
  }

  private void placeLow() 
  {
    if (limitLow.get() == false) 
    {
      liftMotor.set(-.75);
      // If the top limit switch is not pressed, go down
    } 
    else if (limitFront.get() == false) 
    {
      Spike.set(Value.kForward);
      // If the front limit switch is not pressed, move the claw forward
    } 
    else 
    {
      forwardLow = false;
      letsRoll.driveCartesian(0, 0, 0);
      retreatVariable = true;
      encoder1.reset();
      // Don't repeat this method, stop, prepare to pinchAndRetreat
    }
  }

  private void pickup() 
  {
    if (limitLow.get() == false) 
    {
      liftMotor.set(-.36);
      // If the lift isn't in the lowest setting (sensed by limit switch) go down
    } 
    else if (limitFront.get() == false && clawOpen == false) 
    {
      Spike.set(Value.kForward);
      // If the claw isn't forward and isn't open, move the claw forward (and
      // statement explained below)
    } 
    else if (clawOpen == false) 
    {
      clawOpen = true;
      //solenoid.set(true);
      // Makes sure the claw is open
    } else if (distance > -10) 
    {
      letsRoll.driveCartesian(0, -.5, 0);
      // Moves backwards until the encoder is low enough
    } 
    else if (limitBack.get() == false) 
    {
      Spike.set(Value.kReverse);
      // If the claw isn't in the back position, move back. If we didn't have the and
      // statement, it would get stuck going back and forth between these statements
    } 
    else 
    {
      forwardPickup = false;
      // Don't repeat this method
    }
  }

  private void pinchAndRetreat() 
  {
    clawOpen = false;
    solenoid.set(false);
    if (distance > -10) 
    {
      letsRoll.driveCartesian(0, -.5, 0);
      // Move back until the encoder gets to -10
    } else 
    {
      retreatVariable = false;
      // Don't repeat this method
    }
  }

  private void climbByFlipping() 
  {
    encoder1.reset();

    // Move claw back
    // Put "flippers" down
    // Spin right and left side wheels, and normal drive
    // When at the top, stop
  }

  private void Normal() 
  {
    if (Xbox.getXButtonPressed()) 
    {
      rotationButtonTop = true;
    } // starts panel place on top
    if (Xbox.getYButtonPressed()) 
    {
      rotationButtonMid = true;
    } // starts panel place on Mid
    if (Xbox.getBButtonPressed()) 
    {
      rotationButtonLow = true;
    } // starts panel place on Low
    if (Xbox.getAButtonPressed()) 
    {
      panelPickupButton = true;
    } // starts panel pickup
    /*
     * if (Xbox.getRawButtonPressed(5)) { flippyBoi = true; }
     */
    if (Xbox.getY(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) > .4 || Xbox.getX(Hand.kLeft) < -.4
        || Xbox.getY(Hand.kLeft) < -.4) 
    {
          // Jumps out of semiautonomous if joystick is moved far enough
      System.out.println("Failsafe activated");
      rotationButtonLow = false;
      rotationButtonMid = false;
      rotationButtonTop = false;
      panelPickupButton = false;
      forwardTop = false;
      forwardMid = false;
      forwardLow = false;
      forwardPickup = false;
      retreatVariable = false;
      flippyBoi = false;
    }
    if (isEnabled()) 
    {
      System.out.println("Line 407: rotationbuttonTop = " + rotationButtonTop + ", rotationbuttonMid = "
      + rotationButtonMid + ", rotationbuttonLow = " + rotationButtonLow + ", panelPickupButton = "
      + panelPickupButton + ", flipper = " + flippyBoi);
      System.out.println("Line 408: forwardTop = " + forwardTop + ", forwardMid = " + forwardMid + ", forwardLow = "
      + forwardLow + ", forwardPickup = " + forwardPickup + ", retreatVariable = " + retreatVariable
      + ", flippyBoi = " + flippyBoi);
    }
    // printWriter.printf("Line 407: rotationbuttonTop = " + rotationButtonTop + ",
    // rotationbuttonMid = " + rotationButtonMid + ", rotationbuttonLow = " +
    // rotationButtonLow + ", panelPickupButton = " + panelPickupButton + ", flipper
    // = " + flippyBoi);
    
    if (rotationButtonTop == true || rotationButtonMid == true || rotationButtonLow == true || panelPickupButton == true) 
    {
      // If any buttons are true, go to autoCorrect
      // They go below because they are different variables
      camx = tx.getDouble(0.0);
      camy = ty.getDouble(0.0);
      camarea = ta.getDouble(0.0);
      targetRotation = ts.getDouble(0.0);
      autoCorrect();
    } 
    else if (forwardTop == true) 
    {
      placeTop();
      // In these and the following, we go to the desired method
    } 
    else if (forwardMid == true) 
    {
      placeMid();
    } 
    else if (forwardLow == true) 
    {
      placeLow();
    } 
    else if (forwardPickup == true) 
    {
      pickup();
    } 
    else if (retreatVariable == true) 
    {
      pinchAndRetreat();
    } 
    else if (flippyBoi == true) 
    {
      climbByFlipping();
<<<<<<< HEAD
    } else {
      if (Xbox.getY(Hand.kLeft) < .5 || Xbox.getX(Hand.kLeft) < .5 || Xbox.getX(Hand.kLeft) > -.5 || Xbox.getY(Hand.kLeft) > -.5) {
        letsRoll.driveCartesian(Xbox.getX(Hand.kLeft) * .5, Xbox.getY(Hand.kLeft) * -.5, Xbox.getX(Hand.kRight) * .5, 0.0);
         } else {
          letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0);
        }
      // gives us control with more precision at low speed
    }
  }
  private void manualOverride() {
    if (Xbox.getY(Hand.kLeft) < .5 || Xbox.getX(Hand.kLeft) < .5 || Xbox.getX(Hand.kLeft) > -.5 || Xbox.getY(Hand.kLeft) > -.5) {
      letsRoll.driveCartesian(Xbox.getX(Hand.kLeft) * .5, Xbox.getY(Hand.kLeft) * -.5, Xbox.getX(Hand.kRight) * .5, 0.0);
       } else {
        letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0);
      // More precision at low speed
      }
    if (Xbox.getAButton() && limitFront.get() == false) {
=======
    } 
    else 
    {
      letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0);
      // gives us control
    }
  }

  private void manualOverride() 
  {
    letsRoll.driveCartesian(Xbox.getX(Hand.kLeft), Xbox.getY(Hand.kLeft) * -1, Xbox.getX(Hand.kRight), 0.0);
    if (Xbox.getAButton() && limitFront.get() == false) 
    {
>>>>>>> bbae0444c591b4f32ebf5ac2d842387a3291d157
      Spike.set(Relay.Value.kForward);
    }
    else if (Xbox.getBButton() && limitBack.get() == false) 
    {
      Spike.set(Relay.Value.kReverse);
    }
    else 
    {
      Spike.set(Relay.Value.kOff);
    }

    if (Xbox.getTriggerAxis(Hand.kRight) > 0 && limitTop.get() == false) 
    {
      liftMotor.set(Xbox.getTriggerAxis(Hand.kRight));
    } 
    else if (Xbox.getTriggerAxis(Hand.kLeft) > 0 && limitLow.get() == false) 
    {
      liftMotor.set(-Xbox.getTriggerAxis(Hand.kLeft) / 2);
    } 
    else 
    {
      liftMotor.set(0);
    }

    if (Xbox.getRawButton(5)) 
    {
      clawOpen = true;
    } 
    else if (Xbox.getRawButton(6)) 
    {
      clawOpen = false;
    }

    if (Xbox.getYButton() && limitWheelBack.get() == false) 
    {
      liftBot.set(.5);
      liftBotSpinLeft.set(-.5);
      liftBotSpinRight.set(.5);
    } 
    else if (Xbox.getYButton() && limitWheelBack.get() == true) 
    {
      liftBotSpinLeft.set(-.5);
      liftBotSpinRight.set(.5);
    } 
    else if (Xbox.getXButton() && limitWheelFront.get() == false) 
    {
      liftBot.set(-.5);
    } 
    else 
    {
      liftBot.set(0);
      liftBotSpinLeft.set(0);
      liftBotSpinRight.set(0);
    }
  }
}
