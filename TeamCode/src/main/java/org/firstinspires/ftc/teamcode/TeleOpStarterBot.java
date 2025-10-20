package org.firstinspires.ftc.teamcode;

// Import statements - these bring in the FTC SDK classes we need
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;  // Base class for linear (step-by-step) op modes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;         // Annotation to mark this as a TeleOp program
import com.qualcomm.robotcore.hardware.CRServo;               // Continuous rotation servo class
import com.qualcomm.robotcore.hardware.DcMotor;               // Standard DC motor class
import com.qualcomm.robotcore.hardware.DcMotorEx;             // Extended DC motor class with velocity control

// @TeleOp annotation tells the FTC app that this is a driver-controlled program
// It will appear in the TeleOp menu on the Driver Station
@TeleOp
public class TeleOpStarterBot extends LinearOpMode {

  // Hardware declarations - these represent the physical motors and servos on the robot
  // "private" means only this class can use these variables
  private DcMotor flywheel;    // Motor that spins wheels to launch balls
  private DcMotor coreHex;     // Motor that feeds balls into the flywheel (intake/feeder system)
  private DcMotor leftDrive;   // Left drive motor for moving the robot
  private CRServo servo;       // Continuous rotation servo (agitator) in the ball hopper
  private DcMotor rightDrive;  // Right drive motor for moving the robot

  // Velocity target constants - these set how fast the flywheel spins for different shots
  // "static final" means these values never change and belong to the class (not individual objects)
  // Values are in encoder ticks per second - higher numbers = faster spinning
  private static final int bankVelocity = 1300;  // Speed for close shots near the goal
  private static final int farVelocity = 1900;   // Speed for shots from farther away
  private static final int maxVelocity = 2200;   // Maximum flywheel speed

  // @Override means we're replacing the runOpMode method from the parent LinearOpMode class
  // This method runs when the driver presses "INIT" and then "PLAY" on the Driver Station
  @Override
  public void runOpMode() {
    
    // HARDWARE INITIALIZATION SECTION
    // Connect our motor/servo variables to the actual hardware on the robot
    // The strings ("flywheel", "coreHex", etc.) must match the names in the robot configuration
    flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    coreHex = hardwareMap.get(DcMotor.class, "coreHex");
    leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
    servo = hardwareMap.get(CRServo.class, "servo");
    rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

    // MOTOR CONFIGURATION SECTION
    // Set up how the motors should behave
    
    // Enable encoder feedback for precise flywheel speed control
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    // Set motor directions - REVERSE means positive power makes motor spin backwards
    // This is needed because some motors are mounted backwards on the robot
    flywheel.setDirection(DcMotor.Direction.REVERSE);
    coreHex.setDirection(DcMotor.Direction.REVERSE);
    leftDrive.setDirection(DcMotor.Direction.REVERSE);
    
    // Initialize servo to stopped position (0 power = no movement)
    servo.setPower(0);
   
    // Wait for the driver to press PLAY on the Driver Station
    // The robot is now initialized but won't move until PLAY is pressed
    waitForStart();
    
    // Check if the OpMode is still active (hasn't been stopped)
    if (opModeIsActive()) {
      
      // MAIN CONTROL LOOP
      // This loop runs continuously while the OpMode is active
      // It reads gamepad inputs and controls the robot accordingly
      while (opModeIsActive()) {
        
        // Call our custom methods to handle different robot functions
        splitStickArcadeDrive();        // Handle driving the robot around
        setFlywheelVelocity();          // Handle flywheel speed control and shooting
        manualCoreHexAndServoControl(); // Handle manual intake and agitator control
        
        // TELEMETRY SECTION
        // Send data back to the Driver Station screen so drivers can see what's happening
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.update(); // Actually send the telemetry data to the Driver Station
      }
    }
  }

  /**
   * SPLIT STICK ARCADE DRIVE METHOD
   * Controls how the robot moves around the field
   * 
   * Split Stick means:
   * - Left joystick Y-axis: Forward/backward movement
   * - Right joystick X-axis: Left/right turning
   * 
   * This is different from tank drive where each stick controls one side of the robot
   */
  private void splitStickArcadeDrive() {
    float x; // Turning input (left/right)
    float y; // Forward/backward input

    // Get joystick inputs from gamepad 1
    x = gamepad1.right_stick_x;  // Right stick left/right for turning
    y = -gamepad1.left_stick_y;  // Left stick up/down for forward/back (negative because Y is inverted)

    // Calculate motor powers using arcade drive math
    // For arcade drive: left motor = forward - turn, right motor = forward + turn
    leftDrive.setPower(y - x);   // Left motor: forward/back minus turning
    rightDrive.setPower(y + x);  // Right motor: forward/back plus turning
  }
  
  /**
   * MANUAL CONTROL METHOD
   * Handles individual control of the intake system and ball agitator
   * This gives drivers precise control when automatic modes aren't working right
   */
  private void manualCoreHexAndServoControl() {
    
    // Manual control for the Core Hex intake motor
    // This motor feeds balls from the hopper into the flywheel
    if (gamepad1.cross) {           // X button (cross) - feed balls forward
      coreHex.setPower(0.5);        // Run at half speed forward
    } else if (gamepad1.triangle) { // Triangle button - reverse the feeder
      coreHex.setPower(-0.5);       // Run at half speed backward (unjam balls)
    }
    // If neither button is pressed, the motor stops (handled in setFlywheelVelocity method)
    
    // Manual control for the hopper's agitator servo
    // This servo stirs up balls in the hopper to prevent jams
    if (gamepad1.dpad_left) {       // D-pad left - spin servo one direction
      servo.setPower(1);            // Full speed in one direction
    } else if (gamepad1.dpad_right) { // D-pad right - spin servo other direction
      servo.setPower(-1);           // Full speed in opposite direction
    }
    // If neither D-pad button is pressed, servo stops (handled in setFlywheelVelocity method)
  }
  
  /**
   * FLYWHEEL CONTROL METHOD
   * This is the main shooting control system with multiple modes:
   * 
   * MANUAL MODES (flywheel only):
   * - Circle button: Spin flywheel at bank shot speed (close shots)
   * - Square button: Spin flywheel at maximum speed (far shots)
   * 
   * AUTOMATIC MODES (flywheel + feeder + agitator):
   * - Left bumper: Full auto far shot sequence
   * - Right bumper: Full auto bank shot sequence
   * 
   * UTILITY:
   * - Options button: Reverse flywheel (clear jams)
   */
  private void setFlywheelVelocity() {
    
    if (gamepad1.options) {         // Options button - reverse flywheel to clear jams
      flywheel.setPower(-0.5);      // Run flywheel backwards at half power
      
    } else if (gamepad1.left_bumper) {    // Left bumper - automatic far shot
      farPowerAuto();               // Call the far shot automatic sequence
      
    } else if (gamepad1.right_bumper) {   // Right bumper - automatic bank shot
      bankShotAuto();               // Call the bank shot automatic sequence
      
    } else if (gamepad1.circle) {         // Circle button - manual bank shot speed
      ((DcMotorEx) flywheel).setVelocity(bankVelocity); // Set flywheel to bank shot speed only
      
    } else if (gamepad1.square) {         // Square button - manual maximum speed
      ((DcMotorEx) flywheel).setVelocity(maxVelocity);  // Set flywheel to max speed only
      
    } else {
      // NO BUTTONS PRESSED - Stop everything
      ((DcMotorEx) flywheel).setVelocity(0);  // Stop the flywheel
      coreHex.setPower(0);                    // Stop the feeder motor
      
      // Only stop the servo if it's not being manually controlled
      // This prevents the servo from stuttering when drivers use manual D-pad controls
      if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
        servo.setPower(0);  // Stop the agitator servo
      }
    }
  }

  /**
   * BANK SHOT AUTOMATIC SEQUENCE
   * Used for close-range shots when the robot is near the goal
   * 
   * This method coordinates all three shooting components:
   * 1. Spins flywheel to the right speed for close shots
   * 2. Runs agitator to keep balls moving in hopper
   * 3. Waits for flywheel to reach speed, then feeds balls
   * 
   * The sequence prevents feeding balls before the flywheel is ready,
   * which would result in weak or inconsistent shots
   */
  private void bankShotAuto() {
    // Step 1: Start spinning the flywheel at bank shot speed
    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
    
    // Step 2: Start the agitator servo to keep balls moving in the hopper
    servo.setPower(-1);
    
    // Step 3: Only feed balls when flywheel is up to speed
    // Check if current velocity is close enough to target (within 50 ticks/second)
    if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
      coreHex.setPower(1);    // Flywheel ready - start feeding balls
    } else {
      coreHex.setPower(0);    // Flywheel not ready - wait before feeding
    }
  }

  /**
   * FAR SHOT AUTOMATIC SEQUENCE
   * Used for long-range shots when the robot is farther from the goal
   * 
   * This method works like bankShotAuto() but with higher flywheel speed
   * and a larger tolerance for velocity (100 ticks instead of 50)
   * 
   * The larger tolerance accounts for the higher target velocity -
   * it takes longer to reach farVelocity, so we're more generous
   * about when to start feeding balls
   * 
   * Note: May require adjusting the deflector (physical aim mechanism)
   * for accurate shots at this power level
   */
  private void farPowerAuto() {
    // Step 1: Start spinning the flywheel at far shot speed (higher than bank shot)
    ((DcMotorEx) flywheel).setVelocity(farVelocity);
    
    // Step 2: Start the agitator servo to keep balls moving in the hopper
    servo.setPower(-1);
    
    // Step 3: Only feed balls when flywheel is close to target speed
    // Larger tolerance (100 ticks) because higher speeds take longer to reach
    if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
      coreHex.setPower(1);    // Flywheel ready - start feeding balls
    } else {
      coreHex.setPower(0);    // Flywheel not ready - wait before feeding
    }
  }

}
