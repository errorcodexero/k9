#include "WPILib.h"

#define SYNC_GROUP 0x40

const double minSpeed = 1000.;
const double maxSpeed = 3500.;

class ShootyDogThing : public IterativeRobot
{
    Compressor *compressor;
    CANJaguar *topWheel, *bottomWheel;
    DoubleSolenoid *injector;
    Joystick *gamepad;
    double kP, kI, kD;
    double topSpeed, bottomSpeed;
    int report;

public:
    ShootyDogThing():
	compressor(NULL),
	topWheel(NULL),
	bottomWheel(NULL),
	injector(NULL),
	gamepad(NULL),
	kP(1.000),
	kI(0.005),
	kD(0.000),
	topSpeed(1200.),
	bottomSpeed(2800.),
	report(0)
    {
	this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
    }

    ~ShootyDogThing()
    {
	delete gamepad;
	delete injector;
	delete bottomWheel;
	delete topWheel;
	delete compressor;
    }
    
    /**
     * Robot-wide initialization code should go here.
     * 
     * Use this method for default Robot-wide initialization which will
     * be called when the robot is first powered on.  It will be called exactly 1 time.
     */
    void ShootyDogThing::RobotInit() {

	compressor  = new Compressor(1, 1);

	topWheel    = new CANJaguar(5);
	topWheel->SetSafetyEnabled(false);	// motor safety off while configuring

	bottomWheel = new CANJaguar(6);
	bottomWheel->SetSafetyEnabled(false);	// motor safety off while configuring

	injector    = new DoubleSolenoid(1, 2);
	
	gamepad     = new Joystick(1);

	LiveWindow *lw = LiveWindow::GetInstance();
	lw->AddActuator("K9", "Compressor", compressor);
	lw->AddActuator("K9", "Top",        topWheel);
	lw->AddActuator("K9", "Bottom",     bottomWheel);
	lw->AddActuator("K9", "Injector",   injector);

	SmartDashboard::PutNumber("Shooter P", kP);
	SmartDashboard::PutNumber("Shooter I", kI);
	SmartDashboard::PutNumber("Shooter D", kD);

	SmartDashboard::PutNumber("Top Speed",    topSpeed);
	SmartDashboard::PutNumber("Top Voltage",  0.0);
	SmartDashboard::PutNumber("Top Measured", 0.0);

	SmartDashboard::PutNumber("Bottom Speed",    bottomSpeed);
	SmartDashboard::PutNumber("Bottom Voltage",  0.0);
	SmartDashboard::PutNumber("Bottom Measured", 0.0);
    }

    /**
     * Initialization code for disabled mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters disabled mode. 
     */
    void ShootyDogThing::DisabledInit() {

	SmartDashboard::PutNumber("Top Speed", 0.0);
	SmartDashboard::PutNumber("Bottom Speed", 0.0);

	topWheel->Set(0.0, SYNC_GROUP);
	bottomWheel->Set(0.0, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel->DisableControl();
	topWheel->SetSafetyEnabled(false);
	bottomWheel->DisableControl();
	bottomWheel->SetSafetyEnabled(false);

	compressor->Stop();
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

	compressor->Start();
	injector->Set(DoubleSolenoid::kReverse);

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
	SmartDashboard::PutNumber("Top Speed", topSpeed);
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
	if (gamepad->GetRawButton(1)) {
	    topSpeed = minSpeed + (maxSpeed - minSpeed) * (1.0 - gamepad->GetRawAxis(2)) / 2.0;
	    SmartDashboard::PutNumber("Top Speed", topSpeed);
	} else {
	    topSpeed = SmartDashboard::GetNumber("Top Speed");
	}

	if (gamepad->GetRawButton(3)) {
	    bottomSpeed = minSpeed + (maxSpeed - minSpeed) * (1.0 - gamepad->GetRawAxis(2)) / 2.0;
	    SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);
	} else {
	    bottomSpeed = SmartDashboard::GetNumber("Bottom Speed");
	}

	topWheel->Set(topSpeed, SYNC_GROUP);
	bottomWheel->Set(bottomSpeed, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	if (++report >= 25) {	// 500 milliseconds
	    // Update PID parameters
	    kP = SmartDashboard::GetNumber("Shooter P");
	    kI = SmartDashboard::GetNumber("Shooter I");
	    kD = SmartDashboard::GetNumber("Shooter D");
	    topWheel->SetPID( kP, kI, kD );
	    bottomWheel->SetPID( kP, kI, kD );

	    // Get current output voltage and measured speed
	    double topV = topWheel->GetOutputVoltage();
	    double topMeasured = topWheel->GetSpeed(); 
	    double bottomV = bottomWheel->GetOutputVoltage();
	    double bottomMeasured = bottomWheel->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Top Voltage", topV);
	    SmartDashboard::PutNumber("Top Measured", topMeasured);
	    SmartDashboard::PutNumber("Bottom Voltage", bottomV);
	    SmartDashboard::PutNumber("Bottom Measured", bottomMeasured);

	    report = 0;
	}

	if (gamepad->GetRawButton(4))
	{
	    injector->Set(DoubleSolenoid::kForward);
	}
	else if (gamepad->GetRawButton(2))
	{
	    injector->Set(DoubleSolenoid::kReverse);
	}
	else
	{
	    injector->Set(DoubleSolenoid::kOff);
	}

    }

    /**
     * Initialization code for test mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters test mode.
     */
    void ShootyDogThing::TestInit() {

	compressor->Start();
	injector->Set(DoubleSolenoid::kOff);

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

