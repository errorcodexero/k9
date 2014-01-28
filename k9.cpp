#include "WPILib.h"

#define SYNC_GROUP 0x40

const double minSpeed = 1000.;
const double maxSpeed = 3500.;

class ShootyDogThing : public IterativeRobot
{
    CANJaguar *topWheel, *bottomWheel;
    Joystick *gamepad;
    double kP, kI, kD;
    double topSpeed, bottomSpeed;
    int report;

public:
    ShootyDogThing():
	topWheel(NULL),
	bottomWheel(NULL),
	gamepad(NULL),
	kP(1.000),
	kI(0.005),
	kD(0.000),
	topSpeed(0.000),
	bottomSpeed(0.000),
	report(0)
    {
	this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
    }

    ~ShootyDogThing()
    {
	delete gamepad;
	delete bottomWheel;
	delete topWheel;
    }
    
    /**
     * Robot-wide initialization code should go here.
     * 
     * Use this method for default Robot-wide initialization which will
     * be called when the robot is first powered on.  It will be called exactly 1 time.
     */
    void ShootyDogThing::RobotInit() {
	topWheel = new CANJaguar(5);
	bottomWheel = new CANJaguar(6);
	gamepad = new Joystick(1);

	LiveWindow *lw = LiveWindow::GetInstance();
	lw->AddActuator("K9", "Top", topWheel);
	lw->AddActuator("K9", "Bottom", bottomWheel);

	topWheel->SetSafetyEnabled(false);	// motor safety off while configuring
	bottomWheel->SetSafetyEnabled(false);	// motor safety off while configuring

	SmartDashboard::PutNumber("Shooter P", kP);
	SmartDashboard::PutNumber("Shooter I", kI);
	SmartDashboard::PutNumber("Shooter D", kD);

	SmartDashboard::PutNumber("Top Speed", topSpeed);
	SmartDashboard::PutNumber("Top Voltage", 0.0);
	SmartDashboard::PutNumber("Top RPM", 0.0);

	SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);
	SmartDashboard::PutNumber("Bottom Voltage", 0.0);
	SmartDashboard::PutNumber("Bottom RPM", 0.0);
    }

    /**
     * Initialization code for disabled mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters disabled mode. 
     */
    void ShootyDogThing::DisabledInit() {
	topSpeed = 0.0;
	SmartDashboard::PutNumber("Top Speed", topSpeed);
	bottomSpeed = 0.0;
	SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);

	topWheel->Set(topSpeed, SYNC_GROUP);
	bottomWheel->Set(bottomSpeed, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel->DisableControl();
	topWheel->SetSafetyEnabled(false);
	bottomWheel->DisableControl();
	bottomWheel->SetSafetyEnabled(false);
    }

    /**
     * Periodic code for disabled mode should go here.
     * 
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in disabled mode.
     */
    void ShootyDogThing::DisabledPeriodic() {
    }

    /**
     * Initialization code for autonomous mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters autonomous mode.
     */
    void ShootyDogThing::AutonomousInit() {
    }

    /**
     * Periodic code for autonomous mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in autonomous mode.
     */
    void ShootyDogThing::AutonomousPeriodic() {
    }

    /**
     * Initialization code for teleop mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters teleop mode.
     */
    void ShootyDogThing::TeleopInit() {
	// Set control mode
	topWheel->ChangeControlMode( CANJaguar::kSpeed );
	bottomWheel->ChangeControlMode( CANJaguar::kSpeed );

	// Set encoder as reference device for speed controller mode:
	topWheel->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );

	// Set codes per revolution parameter:
	topWheel->ConfigEncoderCodesPerRev( 1 );
	bottomWheel->ConfigEncoderCodesPerRev( 1 );

	// Set Jaguar PID parameters:
	kP = SmartDashboard::GetNumber("Shooter P");
	kI = SmartDashboard::GetNumber("Shooter I");
	kD = SmartDashboard::GetNumber("Shooter D");
	topWheel->SetPID( kP, kI, kD );
	bottomWheel->SetPID( kP, kI, kD );

	// Enable Jaguar control:
	// Increase motor safety timer to something suitably long
	// Poke the motor speed to reset the watchdog, then enable the watchdog
	topSpeed = 1200.0;
	SmartDashboard::PutNumber("Top Speed", topSpeed);
	bottomSpeed = 3200.0;
	SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);

	topWheel->EnableControl();
	topWheel->SetExpiration(2.0);

	bottomWheel->EnableControl();
	bottomWheel->SetExpiration(2.0);

	topWheel->Set(topSpeed, SYNC_GROUP);
	bottomWheel->Set(bottomSpeed, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);
	topWheel->SetSafetyEnabled(true);
	bottomWheel->SetSafetyEnabled(true);

	// reset reporting counter
	report = 0;
    }


    /**
     * Periodic code for teleop mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in teleop mode.
     */
    void ShootyDogThing::TeleopPeriodic() {
	if (gamepad->GetRawButton(4)) {		// "Y"
	    topSpeed = minSpeed + (maxSpeed - minSpeed) * (1.0 - gamepad->GetRawAxis(2)) / 2.0;
	    SmartDashboard::PutNumber("Top Speed", topSpeed);
	} else {
	    topSpeed = SmartDashboard::GetNumber("Top Speed");
	}
	if (gamepad->GetRawButton(1)) {		// "A"
	    bottomSpeed = minSpeed + (maxSpeed - minSpeed) * (1.0 - gamepad->GetRawAxis(2)) / 2.0;
	    SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);
	} else {
	    bottomSpeed = SmartDashboard::GetNumber("Bottom Speed");
	}
	topWheel->Set(topSpeed, SYNC_GROUP);
	bottomWheel->Set(bottomSpeed, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);
	if (++report >= 25) {	// 500 milliseconds
	    double topV = topWheel->GetOutputVoltage();
	    double topRPM = topWheel->GetSpeed(); 
	    double bottomV = bottomWheel->GetOutputVoltage();
	    double bottomRPM = bottomWheel->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Top Voltage", topV);
	    SmartDashboard::PutNumber("Top RPM", topRPM);
	    SmartDashboard::PutNumber("Bottom Voltage", bottomV);
	    SmartDashboard::PutNumber("Bottom RPM", bottomRPM);

	    // Update PID parameters
	    kP = SmartDashboard::GetNumber("Shooter P");
	    kI = SmartDashboard::GetNumber("Shooter I");
	    kD = SmartDashboard::GetNumber("Shooter D");
	    topWheel->SetPID( kP, kI, kD );
	    bottomWheel->SetPID( kP, kI, kD );

	    report = 0;
	}
    }

    /**
     * Initialization code for test mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters test mode.
     */
    void ShootyDogThing::TestInit() {
	topWheel->ChangeControlMode( CANJaguar::kPercentVbus );
	bottomWheel->ChangeControlMode( CANJaguar::kPercentVbus );

	topWheel->EnableControl();
	topWheel->SetExpiration(2.0);

	bottomWheel->EnableControl();
	bottomWheel->SetExpiration(2.0);

	topWheel->Set(0.0, SYNC_GROUP);
	bottomWheel->Set(0.0, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel->SetSafetyEnabled(true);
	bottomWheel->SetSafetyEnabled(true);
    }

    /**
     * Periodic code for test mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in test mode.
     */
    void ShootyDogThing::TestPeriodic() {
    }

};

START_ROBOT_CLASS(ShootyDogThing);

