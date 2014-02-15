#include "WPILib.h"

#define SYNC_GROUP 0x40

const double minSpeed = 1000.;
const double maxSpeed = 3500.;

class ShootyDogThing : public IterativeRobot
{
    Compressor *compressor;
    CANJaguar *topWheel1, *topWheel2;
    CANJaguar *bottomWheel1, *bottomWheel2;
    DoubleSolenoid *injectorL, *injectorR;
    Joystick *gamepad;
    double kP, kI, kD;
    double topSpeed, bottomSpeed;
    int report;

public:
    ShootyDogThing():
	compressor(NULL),
	topWheel1(NULL),
	topWheel2(NULL),
	bottomWheel1(NULL),
	bottomWheel2(NULL),
	injectorL(NULL),
	injectorR(NULL),
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
	delete injectorR;
	delete injectorL;
	delete bottomWheel2;
	delete bottomWheel1;
	delete topWheel2;
	delete topWheel1;
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

	topWheel1    = new CANJaguar(3);
	topWheel1->SetSafetyEnabled(false);	// motor safety off while configuring

	topWheel2    = new CANJaguar(3);
	topWheel2->SetSafetyEnabled(false);	// motor safety off while configuring

	bottomWheel1 = new CANJaguar(5);
	bottomWheel1->SetSafetyEnabled(false);	// motor safety off while configuring

	bottomWheel2 = new CANJaguar(6);
	bottomWheel2->SetSafetyEnabled(false);	// motor safety off while configuring

	injectorL    = new DoubleSolenoid(1, 2);
	injectorR    = new DoubleSolenoid(3, 4);
	
	gamepad      = new Joystick(1);

	LiveWindow *lw = LiveWindow::GetInstance();
	lw->AddActuator("K9", "Compressor", compressor);
	lw->AddActuator("K9", "Top1",       topWheel1);
	lw->AddActuator("K9", "Top2",       topWheel2);
	lw->AddActuator("K9", "Bottom1",    bottomWheel1);
	lw->AddActuator("K9", "Bottom2",    bottomWheel2);
	lw->AddActuator("K9", "InjectorL",  injectorL);
	lw->AddActuator("K9", "InjectorR",  injectorR);

	SmartDashboard::PutNumber("Shooter P", kP);
	SmartDashboard::PutNumber("Shooter I", kI);
	SmartDashboard::PutNumber("Shooter D", kD);

	SmartDashboard::PutNumber("Top Speed",     topSpeed);
	SmartDashboard::PutNumber("Top Current 1", 0.0);
	SmartDashboard::PutNumber("Top Current 2", 0.0);
	SmartDashboard::PutNumber("Top Measured",  0.0);

	SmartDashboard::PutNumber("Bottom Speed",     bottomSpeed);
	SmartDashboard::PutNumber("Bottom Current 1", 0.0);
	SmartDashboard::PutNumber("Bottom Current 2", 0.0);
	SmartDashboard::PutNumber("Bottom Measured",  0.0);
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

	topWheel1->Set(0.0, SYNC_GROUP);
	topWheel2->Set(0.0, SYNC_GROUP);
	bottomWheel1->Set(0.0, SYNC_GROUP);
	bottomWheel2->Set(0.0, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel1->DisableControl();
	topWheel1->SetSafetyEnabled(false);

	topWheel2->DisableControl();
	topWheel2->SetSafetyEnabled(false);

	bottomWheel1->DisableControl();
	bottomWheel1->SetSafetyEnabled(false);

	bottomWheel2->DisableControl();
	bottomWheel2->SetSafetyEnabled(false);

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
	injectorL->Set(DoubleSolenoid::kReverse);
	injectorR->Set(DoubleSolenoid::kReverse);

	// Set control mode
	topWheel1->ChangeControlMode( CANJaguar::kSpeed );
	topWheel2->ChangeControlMode( CANJaguar::kSpeed );
	bottomWheel1->ChangeControlMode( CANJaguar::kSpeed );
	bottomWheel2->ChangeControlMode( CANJaguar::kSpeed );

	// Set encoder as reference device for speed controller mode:
	topWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	topWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );

	// Set codes per revolution parameter:
	topWheel1->ConfigEncoderCodesPerRev( 1 );
	topWheel2->ConfigEncoderCodesPerRev( 1 );
	bottomWheel1->ConfigEncoderCodesPerRev( 1 );
	bottomWheel2->ConfigEncoderCodesPerRev( 1 );

	// Set Jaguar PID parameters:
	kP = SmartDashboard::GetNumber("Shooter P");
	kI = SmartDashboard::GetNumber("Shooter I");
	kD = SmartDashboard::GetNumber("Shooter D");
	topWheel1->SetPID( kP, kI, kD );
	topWheel2->SetPID( kP, kI, kD );
	bottomWheel1->SetPID( kP, kI, kD );
	bottomWheel2->SetPID( kP, kI, kD );

	// Enable Jaguar control:
	// Increase motor safety timer to something suitably long
	// Poke the motor speed to reset the watchdog, then enable the watchdog
	SmartDashboard::PutNumber("Top Speed", topSpeed);
	SmartDashboard::PutNumber("Bottom Speed", bottomSpeed);

	topWheel1->EnableControl();
	topWheel1->SetExpiration(2.0);

	topWheel2->EnableControl();
	topWheel2->SetExpiration(2.0);

	bottomWheel1->EnableControl();
	bottomWheel1->SetExpiration(2.0);

	bottomWheel2->EnableControl();
	bottomWheel2->SetExpiration(2.0);

	topWheel1->Set(topSpeed, SYNC_GROUP);
	topWheel2->Set(topSpeed, SYNC_GROUP);
	bottomWheel1->Set(bottomSpeed, SYNC_GROUP);
	bottomWheel2->Set(bottomSpeed, SYNC_GROUP);

	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel1->SetSafetyEnabled(true);
	topWheel2->SetSafetyEnabled(true);
	bottomWheel1->SetSafetyEnabled(true);
	bottomWheel2->SetSafetyEnabled(true);

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
	// schedule updates to avoid overloading CAN bus or CPU
	switch (report++) {
	case 0:
	    // Update PID parameters
	    double newP = SmartDashboard::GetNumber("Shooter P");
	    double newI = SmartDashboard::GetNumber("Shooter I");
	    double newD = SmartDashboard::GetNumber("Shooter D");
	    if (newP != kP || newI != kI || newD != kD) {
		kP = newP;
		kI = newI;
		kD = newD;
		topWheel1->SetPID( kP, kI, kD );
		topWheel2->SetPID( kP, kI, kD );
		bottomWheel1->SetPID( kP, kI, kD );
		bottomWheel2->SetPID( kP, kI, kD );
	    }
	    break;

	case 8:
	    // Get top output voltage, current and measured speed
	    double topI1 = topWheel1->GetOutputCurrent();
	    double topI2 = topWheel2->GetOutputCurrent();
	    double topMeasured = topWheel1->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Top Current 1", topI1);
	    SmartDashboard::PutNumber("Top Current 2", topI2);
	    SmartDashboard::PutNumber("Top Measured",  topMeasured);
	    break;

	case 16:
	    // Get bottom output voltage, current and measured speed
	    double bottomI1 = bottomWheel1->GetOutputCurrent();
	    double bottomI2 = bottomWheel2->GetOutputCurrent();
	    double bottomMeasured = bottomWheel1->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Bottom Current 1", bottomI1);
	    SmartDashboard::PutNumber("Bottom Current 2", bottomI2);
	    SmartDashboard::PutNumber("Bottom Measured",  bottomMeasured);
	    break;

	case 24:		// 480 milliseconds
	    report = 0;
	}

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

	topWheel1->Set(topSpeed, SYNC_GROUP);
	topWheel2->Set(topSpeed, SYNC_GROUP);
	bottomWheel1->Set(bottomSpeed, SYNC_GROUP);
	bottomWheel2->Set(bottomSpeed, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	if (gamepad->GetRawButton(4))
	{
	    injectorL->Set(DoubleSolenoid::kForward);
	    injectorR->Set(DoubleSolenoid::kForward);
	}
	else if (gamepad->GetRawButton(2))
	{
	    injectorL->Set(DoubleSolenoid::kReverse);
	    injectorR->Set(DoubleSolenoid::kReverse);
	}
	else
	{
	    injectorL->Set(DoubleSolenoid::kOff);
	    injectorR->Set(DoubleSolenoid::kOff);
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
	injectorL->Set(DoubleSolenoid::kOff);
	injectorR->Set(DoubleSolenoid::kOff);

	topWheel1->ChangeControlMode( CANJaguar::kPercentVbus );
	topWheel2->ChangeControlMode( CANJaguar::kPercentVbus );
	bottomWheel1->ChangeControlMode( CANJaguar::kPercentVbus );
	bottomWheel2->ChangeControlMode( CANJaguar::kPercentVbus );

	topWheel1->EnableControl();
	topWheel1->SetExpiration(2.0);

	topWheel2->EnableControl();
	topWheel2->SetExpiration(2.0);

	bottomWheel1->EnableControl();
	bottomWheel1->SetExpiration(2.0);

	bottomWheel2->EnableControl();
	bottomWheel2->SetExpiration(2.0);

	topWheel1->Set(0.0, SYNC_GROUP);
	topWheel2->Set(0.0, SYNC_GROUP);
	bottomWheel1->Set(0.0, SYNC_GROUP);
	bottomWheel2->Set(0.0, SYNC_GROUP);
	CANJaguar::UpdateSyncGroup(SYNC_GROUP);

	topWheel1->SetSafetyEnabled(true);
	topWheel2->SetSafetyEnabled(true);
	bottomWheel1->SetSafetyEnabled(true);
	bottomWheel2->SetSafetyEnabled(true);
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

