#include <mbed.h>
#include <vector>
#include "source/EncodedMotor.h"	// motor encoder
#include "source/DebugMonitor.h"	// enable LCD2004 as debug monitor
#include <TextLCD.h>
#include "source/ShiftReg7Seg.h"
#include "source/MotorControl.h"
#include "source/EventVariable.h"
#include "source/MovingAverage.h"

// set baudrate at mbed_config.h default 115200
// I2C scanner included, derived from Arduino I2C scanner
// http://www.gammon.com.au/forum/?id=10896
/////////////////////////////////
//// Declare connection//////////
/////////////////////////////////
// L298N Control PinName
PinName MotorEnable = PA_15;		// connect ENA to PA_15
PinName MotorDirection1 = PA_14;	// connect IN1 to PA_14
PinName MotorDirection2 = PA_13;	// connect IN2 to PA_13
DigitalOut SolenoidEnable = PB_7;	// connect ENB to PB_7
// User Input PinName
PinName WeldStartStop = PA_11; 		// connect WeldStartStop Btn to PA_11
PinName MotorStartStop = PA_12;		// connect MotorStartStop Btn to PA_12
PinName MotorChangeDirection = PC_13;   // user btn on board, for changing motor direction
PinName knob = PA_4;				// connect Potentionmeter to PA_4 (A2)
// Motor Encoder PinName
PinName MotorEncoderA = PA_9;		// connect MotorEncoderA (blue) to PA_9 (D8)
PinName MotorEncoderB = PA_8;		// connect MotorEncoderB (purple) to PA_8 (D7)

// LED Indicator Port
DigitalOut MotorLED = PB_15;		// connect MotorLED to PB_15
DigitalOut TorchLED = PB_14;		// connect TorchLED to PB_14
// Port Declaration
AnalogIn refSpeed(knob);			// Reference Speed from user through potentiometer
PwmOut MotorEnablePin(MotorEnable);
DigitalOut MotorDirectionPin1(MotorDirection1);
DigitalOut MotorDirectionPin2(MotorDirection2);

//// Initiate object
RawSerial pc(SERIAL_TX, SERIAL_RX, 115200);		                // serial communication protocol
EncodedMotor encoder(MotorEncoderA, MotorEncoderB, 1848*4, 10, EncodeType::X4);		// Encoded Motor object
std::unique_ptr<MotorControl> motor1 = std::make_unique<MotorControl>
        (&MotorEnablePin, &MotorDirectionPin1, &MotorDirectionPin2, &encoder, 0.16, 0.01);		// motor controller object, Kp and Ki specified
DebugMonitor debugger(&refSpeed, &encoder, &pc);		        // update status through LCD2004 and Serial Monitor
ShiftReg7Seg disp1(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS, 4, D9); // 7 segments display

//// Declare interrupt
Ticker statusUpdater;			// Periodic Interrupt for debugging purpose
Ticker motorBlinkLEDTicker;		// LED Blinker
Ticker buttonRestart;          // To cancel out fluctuating button signal
InterruptIn motorBtn(MotorStartStop);		// Motor Button Interrupt
InterruptIn weldingBtn(WeldStartStop);		// Welding Button Interrupt
InterruptIn motorChgDirBtn(MotorChangeDirection);   // Motor Change Direction Interrupt (btn on board)

//// Declare thread
Thread motorLEDBlinking;		// Thread to perform LED Blinking
Thread statusUpdateThread;      // Thread to perform Status Update
Thread dispThread;              // Thread to display 7 segments display

//// Declare event flag
EventFlags statusUpdateFlag;
EventFlags interuptRestartFlag;

//// Define constants
// volatile bool motorStartBtnChange = false;		// Start motor flag
volatile bool prevMotorSteady = false;		// Store previous motor steady state
//volatile float currentSpeed;

//// Fwd declare
void I2C_scan();
void motorStartBtnChangeEvent(bool &);		// Determine motor start status
void motorRunner(); 
void MotorLEDBlinker(bool&);
void buttonRestartEvent();

// Initiate EventVariable
EventVariable<bool> motorStartBtnChange(false, &motorStartBtnChangeEvent);
EventVariable<bool> motorSteadySignal(true, &MotorLEDBlinker);
EventVariable<bool> weldSignal(false,[](){TorchLED != TorchLED; });

//// Define function
void I2C_scan()
{
	// Define I2C communication using port I2C1_SDA(D14, PB_9) and I2C1_SCL(D15, PB_8)
	I2C i2c(PB_9, PB_8);

	// Initializing I2C Scanner
	pc.printf("I2C Scanner initializing.... \n");
	uint8_t error, address;
	char mydata[2];
	mydata[1] = 0x00;
	std::vector<uint8_t> addrList;

	pc.printf("I2C Scanner start scanning. \n");

	// Start scanning using 8 bit address
	pc.printf("\n8-bit address.... \n");
	for (address = 1; address < 128; address++)
	{
		error = i2c.write(address, mydata, sizeof(mydata));					// error: 0 - success (ack), 1 - failure (nack)
		pc.printf("Address: %#X return %d\n", address, error);
		if (error == 0)
			addrList.push_back(address);	// add to addrList if address receive ack
	}
	for (auto &val : addrList)
	{
		pc.printf("Valid Address: %#X ", val); 		// print valid address
	}

	// Initialize and start scanning using 16-bit (8-bit shifted left) address
	addrList.clear();
	pc.printf("\n16-bit address.... \n");
	for (address = 1; address < 128; address++)
	{
		error = i2c.write((uint16_t)(address << 1), mydata, sizeof(mydata));  // error: 0 - success (ack), 1 - failure (nack)
		pc.printf("Address: %#X return %d\n", (uint16_t)(address << 1), error);
		if (error == 0)
			addrList.push_back(address);	// add to addrList if address receive ack
	}
	for (auto &val : addrList)
	{
		pc.printf("Valid Address: %#X ", val);		// print valid address
	}
}
void statusUpdateEvent()
{
    while(1)
    {
        statusUpdateFlag.wait_all(0x1);

        // Output status
        debugger.printSignal();

        // Output Flags to Serial monitor
        pc.printf("motorStartBtnChange: %d\n motorSteadySignal: %d\n weldSignal: %d\n SolenoidEnable = %d\n",
                  motorStartBtnChange.value, motorSteadySignal.value, weldSignal.value, SolenoidEnable.read());
        pc.printf("RefSpeed: %f\n Compensate: %f\n Speed: %f\n Error: %lf\n AdjError: %lf\n Current Direction: %d\n",
                  refSpeed.read()*100, motor1->readComp(), motor1->readSpeed(), motor1->readError(), motor1->readAdjError(),
                  motor1->getCurrentDirection());
    }
}
void motorStartBtnChangeEvent(bool &motorState) {
	// Active-Deactivate Motor Rotation
	if (!motorState) {
		motorBlinkLEDTicker.detach();
		MotorLED = 0;
	}
	else {
		motorSteadySignal = false;
	}

	// disable motor button till next MotorLED blink .... to avoid noisy signal
	motorBtn.disable_irq();         // disable interrupt
	buttonRestart.attach(&buttonRestartEvent,1.0f);
}
void motorRunner() 
{
    bool tempMotorSteady = motor1->run(refSpeed.read());			// run motor1 and read motor1 steady state
	if (tempMotorSteady != prevMotorSteady) motorSteadySignal = tempMotorSteady;
	prevMotorSteady = tempMotorSteady;
}
void motorStopper()
{
	motorSteadySignal = 0; 
	motor1->stop();
}
void MotorLEDBlinker(bool& motorSteady)			// Run motor and set motorOnLED to blinking / solid light
{
	if (motorSteady) { motorBlinkLEDTicker.detach(); MotorLED = 1; }
	else motorBlinkLEDTicker.attach([]() {MotorLED = !MotorLED; }, 0.5f);

}
void displayCurrentSpeed(){
    while(1){
        disp1.display(motor1->readSpeed());
        wait(0.1);
    }
}
void buttonRestartEvent() {motorBtn.enable_irq(); weldingBtn.enable_irq(); buttonRestart.detach(); }
int main() {
	pc.printf("Initiating\n");

	// I2C Scanner.. comment out if not used...
	// I2C_scan();

	// Initiate Interrupt and Ticker
	motorBtn.rise([]() {motorStartBtnChange = !motorStartBtnChange; });				// motorBtn OnChange
	weldingBtn.rise([&]() {
		bool toSolenoid = motorStartBtnChange.value && motorSteadySignal.value && !weldSignal;
		toSolenoid ? weldSignal = true : weldSignal = false; });					// weldingBtn OnChange
    motorChgDirBtn.rise([](){motor1->chgDirection(); });
	statusUpdater.attach([](){statusUpdateFlag.set(0x1); }, 0.5f);					// periodic status update via flag

    // Start Thread
	dispThread.start(displayCurrentSpeed);			// 7-segment Thread Start
    statusUpdateThread.start(&statusUpdateEvent);   // Start Status Update Event

    // Initialize Output
    TorchLED = 0; 								// Initialize TorchLED

	pc.printf("Ready\n");

	while (1) {
		if (motorStartBtnChange.value) {motorRunner();}
		else { motorStopper(); }
	}
}