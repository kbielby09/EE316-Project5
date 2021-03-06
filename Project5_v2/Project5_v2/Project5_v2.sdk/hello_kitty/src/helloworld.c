
#include <stdio.h>
#include "PWM.h"
#include "xsysmon.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xgpio.h"
#include "sleep.h"
#include "xil_types.h"
#include "xtmrctr.h"
#include "xparameters.h"

int on_time = 25000;  // Time that the signal is high in nano seconds
int changed_period = 2000000;  // Period in nano seconds
int motor_duty_cycle = 6000000;
int led_high_time = 0;
u32 Source = 0;

#define TMR_LOAD 0xF8000000
#define INPUT_SCALE 0x0FFFF
#define XADC_DEVICE_ID XPAR_XADC_WIZ_0_DEVICE_ID
#define XADC_SEQ_CHANNELS 0xB3630800
//#define XADC_SEQ_CHANNELS 0xB1610008
#define XADC_CHANNELS 0xB3630008
#define NUMBER_OF_CHANNELS 10
const u8 Channel_List[NUMBER_OF_CHANNELS] = {
	3, // Start with VP/VN
	28, 16, 24, // Diff. Channels in ascending order
	17, 25, 22, 31, 21, 29 // Single-Ended Channels in ascending order
}; // 00008
const char *Channel_Names[32] = {
	"", "", "", "VP-VN",
	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"A8-A9", "A0", "", "",
	"", "A4", "A2", "",
	"A10-A11", "A1", "", "",
	"A6-A7", "A5", "", "A3"
};


#define Test_Bit(VEC,BIT) ((VEC&(1<<BIT))!=0)


#define TMRCTR_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID
#define PWM_PERIOD              20000000    /* PWM period in (500 ms) */
//#define SERVO_PERIOD            20000000
#define SERVO_PERIOD            1000000
#define WAIT_COUNT              10000000

int TmrCtrPwm(XTmrCtr *InstancePtr, u16 DeviceId);
int ResetTimer();
void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId);
float Xadc_RawToVoltage(u16 Data, u8 Channel);
u32 Xadc_ReadData (XSysMon *InstancePtr, u16 RawData[32]);
void Xadc_Demo(XSysMon *InstancePtr, u32 ChannelSelect);

//static void TMR_Intr_Handler(void *baseaddr_p);
//static int IntcInitFunction(u16 DeviceId, XTmrCtr *TrmInstance);

XTmrCtr TimerCounterInst;  /* The instance of the Timer Counter */
XTmrCtr TimerCounterInst_1;  /* The instance of the Timer Counter */
XTmrCtr TimerCounterInst_2;  /* The instance of the Timer Counter */
XTmrCtr TimerCounter_Interrupt ;  /* The instance of the Timer Counter */
XScuGic INTCInst;

/************************** Constant Definitions *****************************/
#ifndef TESTAPP_GEN
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_DEVICE_ID		XPAR_GPIO_1_DEVICE_ID
#define GPIO_CHANNEL1		1

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_GPIO_INTERRUPT_ID	XPAR_INTC_0_GPIO_0_VEC_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
#else
 #define INTC_GPIO_INTERRUPT_ID	XPAR_FABRIC_AXI_GPIO_1_IP2INTC_IRPT_INTR
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#endif /* XPAR_INTC_0_DEVICE_ID */

/*
 * The following constants define the positions of the buttons and LEDs each
 * channel of the GPIO
 */
#define GPIO_ALL_LEDS		0xFFFF
#define GPIO_ALL_BUTTONS	0xFFFF

/*
 * The following constants define the GPIO channel that is used for the buttons
 * and the LEDs. They allow the channels to be reversed easily.
 */
#define BUTTON_CHANNEL	 1	/* Channel 1 of the GPIO Device */
#define LED_CHANNEL	 2	/* Channel 2 of the GPIO Device */
#define BUTTON_INTERRUPT XGPIO_IR_CH1_MASK  /* Channel 1 Interrupt Mask */

/*
 * The following constant determines which buttons must be pressed at the same
 * time to cause interrupt processing to stop and start
 */
#define INTERRUPT_CONTROL_VALUE 0x7

/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */
#define LED_DELAY	1000000

#endif /* TESTAPP_GEN */

#define INTR_DELAY	0x0FFF

#define RS 1    /* BIT0 mask */
#define RW 2    /* BIT1 mask */
#define EN 4    /* BIT2 mask */

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
 #define INTC		XIntc
 #define INTC_HANDLER	XIntc_InterruptHandler
#else
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define INTC		XScuGic
 #define INTC_HANDLER	XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

/************************** Function Prototypes ******************************/
void GpioHandler(void *CallBackRef);

int GpioIntrExample(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId,
			u16 IntrMask, u32 *DataRead);

int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask);

void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask);

void GpioCheckForIntrLoop(u32 *DataRead, u32 *Source);
void GpioCheckForIntr(u32 *DataRead);

void delayMs(int n);
void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);

/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */
XGpio Gpio; /* The Instance of the GPIO Driver */

INTC Intc; /* The Instance of the Interrupt Controller Driver */


static u16 GlobalIntrMask; /* GPIO channel mask that is needed by
			    * the Interrupt Handler */

static volatile u32 IntrFlag; /* Interrupt Handler Flag */
int resetSig = 0;
int sourceSig = 0;
int enaSig = 0;


int tmr_count = 0;

//void TMR_Intr_Handler(void *data){
//  if(XTmrCtr_IsExpired(&TimerCounter_Interrupt, 0)) {
//      if(tmr_count == 3){
//    	  XTmrCtr_Stop(&TimerCounter_Interrupt, 0);
//    	  XTmrCtr_Reset(&TimerCounter_Interrupt, 0);
//    	  XTmrCtr_Start(&TimerCounter_Interrupt, 0);
//      } else
//      {
//    	  tmr_count++;
//      }
//  }
//}
//
//int InterruptSystemSetup(XScuGic * instance_ptr) {
//	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
//			(Xil_ExceptionHandler)XScuGic_InterruptHandler, instance_ptr);
//	Xil_ExceptionEnable();
//
//	return XST_SUCCESS;
//}

//int IntcInitFunction(u16 DeviceId, XTmrCtr *TmrInstancePtr)
//{
//	XScuGic_Config *IntcConfig;
//	int status;
//
//	// Interrupt controller initialization
//	IntcConfig = XScuGic_LookupConfig(DeviceId);
//	status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress);
//
//    status = InterruptSystemSetup(&INTCInst);
//
//    status = XScuGic_Connect(&INTCInst, XPAR_FABRIC_AXI_TIMER_0_INTERRUPT_INTR,
//    		(Xil_ExceptionHandler) TMR_Intr_Handler, (void *) TmrInstancePtr);
//
//    // Enable timer interrupts in the controller
//    XScuGic_Enable(&INTCInst, XPAR_FABRIC_AXI_TIMER_0_INTERRUPT_INTR);
//
//}


int main()
{
	int Status;

	PWM_Set_Period(XPAR_PWM_0_PWM_AXI_BASEADDR, SERVO_PERIOD);
	PWM_Enable(XPAR_PWM_0_PWM_AXI_BASEADDR);

	XSysMon Xadc;
	u8 ChannelIndex = 4;
	u32 time_count = 0;

//	Status = IntcInitFunction(XPAR_PS7_SCUGIC_0_DEVICE_ID, &TimerCounter_Interrupt);
//	Status = XTmrCtr_Initialize(&TimerCounter_Interrupt, XPAR_TMRCTR_3_DEVICE_ID);
//	XTmrCtr_SetHandler(&TimerCounter_Interrupt, (XTmrCtr_Handler) TMR_Intr_Handler, &TimerCounter_Interrupt);
//	XTmrCtr_SetResetValue(&TimerCounter_Interrupt, 0, TMR_LOAD);
//	XTmrCtr_SetOptions(&TimerCounter_Interrupt, 0,
//			XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);
//
//	XTmrCtr_Start(&TimerCounter_Interrupt, 0);


	Xadc_Init(&Xadc, XADC_DEVICE_ID);

	u32 Mode = 0;
//	u32 Source = 0;
	char *str1 = "Enable";
	int str1len = strlen(str1);
	char *str2 = "Disable";
	int str2len = strlen(str2);
	char *str3 = "Resetting...";
	int str3len = strlen(str3);

	char *str4 = "Potentiometer";
	int str4len = strlen(str4);
	char *str5 = "Photo-Resistor";
	int str5len = strlen(str5);

	Status = GpioIntrExample(&Intc, &Gpio,
					   GPIO_DEVICE_ID,
					   INTC_GPIO_INTERRUPT_ID,
					   GPIO_CHANNEL1, &Mode);

	LCD_init();
	LCD_command(1);
	delayMs(500);
	LCD_command(0x80);

	printf("Cora XADC Demo Initialized!\r\n");

	while(1) {
		while(Mode == 1){
			on_time = 0;  // Time that the signal is high in nano seconds
			changed_period = 0;  // Period in nano seconds
			motor_duty_cycle = 0;
			led_high_time = 0;
			LCD_command(0x80); /* The First Line */
			for(int i = 0; i < str2len; i++)
			{
				LCD_data(str2[i]);
			}
			ResetTimer();
//			Source = 0;
			GpioCheckForIntrLoop(&Mode, &Source);
		}
		if(Status == 0 && resetSig != 1)
		{
			LCD_command(0x80); /* The First Line */
			if(Mode == 0)
			{
				for(int i = 0; i < str1len; i++)
				{
					LCD_data(str1[i]);
				}
			}
			else
			{
				printf("Mode == 1\n");
				for(int i = 0; i < str2len; i++)
				{
					LCD_data(str2[i]);
				}
			}

			LCD_command(0xC0); /* The Second Line */
			if(Source == 0)
			{
				for(int i = 0; i < str4len; i++)
				{
					LCD_data(str4[i]);
				}
			}
			else
			{
				for(int i = 0; i < str5len; i++)
				{
					LCD_data(str5[i]);
				}
			}
		}
		else
		{
			printf("disable\n");
			LCD_init();
			LCD_command(1);
			delayMs(500);
			LCD_command(0x80);
			for(int i = 0; i < str3len; i++)
			{
				LCD_data(str3[i]);
			}
			Mode = 0;
			Source = 0;
			delayMs(1000);
			LCD_command(1);
			resetSig = 0;
		}
		GpioCheckForIntrLoop(&Mode, &Source);
//        printf("at counter\n");
		time_count ++;
//		printf("timer count: %d\n", (int)time_count);
		if (time_count == 10) { // print channel reading approx. 10x per second
			time_count = 0;
			Xadc_Demo(&Xadc, Channel_List[ChannelIndex]);
		}
//		usleep(1);
//		printf("end of counter\n");
	}

    return 0;
}

//int TmrCtrPwm(XTmrCtr *TmrCtrInstancePtr, u16 DeviceId)
//{
//	u8  DutyCycle;
//	float DutyCycle_percent;
//	int  Div;
//	u32 Period;
//	u32 HighTime;
//	int Status;

	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
//	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
//	if (Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}

	/* Disable PWM for reconfiguration */
//	XTmrCtr_PwmDisable(TmrCtrInstancePtr);

	/* Configure PWM */
//	Period = PWM_PERIOD;
//	for(Div = 100; Div > 0; Div--){
//		/* Disable PWM for reconfiguration */
//		XTmrCtr_PwmDisable(TmrCtrInstancePtr);
//
//		/* Configure PWM */
//		HighTime = (0.025+0.001*Div)*PWM_PERIOD;
////		DutyCycle = XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period, HighTime);
//
//		DutyCycle_percent = ((float)HighTime /(float)Period)*100;
////		xil_printf("PWM Configured for Period = %d & DutyCyles = %d\r\n", Period, HighTime);
//		printf("PWM percent Duty Cycle = %5.2F\r\n", DutyCycle_percent);
//
//		/* Enable PWM */
//		XTmrCtr_PwmEnable(TmrCtrInstancePtr);
//
//		   for(int Counter = 0; Counter < WAIT_COUNT; Counter++){
//			}
//	}

//	return Status;
//}


int ResetTimer(){
	int Status;
	u8  DutyCycle;

	/*
	* Initialize the timer counter so that it's ready to use,
	* specify the device ID that is generated in xparameters.h
	*/
	Status = XTmrCtr_Initialize(&TimerCounterInst, XPAR_TMRCTR_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Disable PWM for reconfiguration */
	XTmrCtr_PwmDisable(&TimerCounterInst);

	DutyCycle = XTmrCtr_PwmConfigure(&TimerCounterInst, PWM_PERIOD, motor_duty_cycle);

	/* Enable PWM */
    XTmrCtr_PwmEnable(&TimerCounterInst);

    /*
    * Initialize the timer counter so that it's ready to use,
    * specify the device ID that is generated in xparameters.h
    */
    Status = XTmrCtr_Initialize(&TimerCounterInst_1, XPAR_TMRCTR_1_DEVICE_ID);
    if (Status != XST_SUCCESS) {
    	return XST_FAILURE;
    }

    /* Disable PWM for reconfiguration */
    XTmrCtr_PwmDisable(&TimerCounterInst_1);

   	DutyCycle = XTmrCtr_PwmConfigure(&TimerCounterInst_1, changed_period, 0.5 * changed_period);

   	/* Enable PWM */
    XTmrCtr_PwmEnable(&TimerCounterInst_1);

	/*
	* Initialize the timer counter so that it's ready to use,
	* specify the device ID that is generated in xparameters.h
	*/
	Status = XTmrCtr_Initialize(&TimerCounterInst_2, XPAR_TMRCTR_2_DEVICE_ID);
	if (Status != XST_SUCCESS) {
	 	return XST_FAILURE;
	}

	/* Disable PWM for reconfiguration */
	XTmrCtr_PwmDisable(&TimerCounterInst_2);
  	DutyCycle = XTmrCtr_PwmConfigure(&TimerCounterInst_2, PWM_PERIOD, led_high_time);

   	/* Enable PWM */
    XTmrCtr_PwmEnable(&TimerCounterInst_2);

    PWM_Set_Duty(XPAR_PWM_0_PWM_AXI_BASEADDR, on_time, 0);

    return 0;
}

void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId) {
	XSysMon_Config *ConfigPtr;
	ConfigPtr = XSysMon_LookupConfig(DeviceId);
	XSysMon_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress);

	// Disable the Channel Sequencer before configuring the Sequence registers.
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_SAFE);
	// Disable all alarms
	XSysMon_SetAlarmEnables(InstancePtr, 0x0);
	// Set averaging for all channels to 16 samples
	XSysMon_SetAvg(InstancePtr, XSM_AVG_16_SAMPLES);
	// Set differential input mode for all channels
	XSysMon_SetSeqInputMode(InstancePtr, 0);
	// Set 6ADCCLK acquisition time in all channels
	XSysMon_SetSeqAcqTime(InstancePtr, XADC_SEQ_CHANNELS);
	// Disable averaging in all channels
	XSysMon_SetSeqAvgEnables(InstancePtr, XADC_SEQ_CHANNELS);
	// Enable all channels
	XSysMon_SetSeqChEnables(InstancePtr, XADC_SEQ_CHANNELS);
	// Set the ADCCLK frequency equal to 1/32 of System clock
	XSysMon_SetAdcClkDivisor(InstancePtr, 32);
	// Enable Calibration
	XSysMon_SetCalibEnables(InstancePtr, XSM_CFR1_CAL_PS_GAIN_OFFSET_MASK | XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK);
	// Enable the Channel Sequencer in continuous sequencer cycling mode
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_CONTINPASS);
	// Clear the old status
//	XSysMon_GetStatus(InstancePtr);
}

float Xadc_RawToVoltage(u16 Data, u8 Channel) {
	float FloatData;
	float Scale;

	switch (Channel) {
	case 3: // VP/VN (Cora Dedicated Analog Input)
	case 16: // AUX0 (Cora A8/A9 Diff. Analog Input)
	case 24: // AUX8 (Cora A10/A11 Diff. Analog Input)
	case 28: Scale = 1.0; break; // AUX12 (Cora A6/A7 Diff. Analog Input)
	case 17: // AUX1 (Cora A0 Single-Ended Analog Input)
	case 21: // AUX5 (Cora A4 Single-Ended Analog Input)
	case 22: // AUX6 (Cora A2 Single-Ended Analog Input)
	case 25: // AUX9 (Cora A1 Single-Ended Analog Input)
	case 29: // AUX13 (Cora A5 Single-Ended Analog Input)
	case 31: Scale = 3.3; break; // AUX15 (Cora A3 Single-Ended Analog Input)
	default: Scale = 0.0;
	}
	if (Test_Bit(Data, 15)) {
		FloatData = -Scale;
		Data = ~Data + 1;
	} else
		FloatData = Scale;
	FloatData *= (float)Data / (float)0xFFFF;
	return FloatData;
}

#define READDATA_DBG 0
u32 Xadc_ReadData (XSysMon *InstancePtr, u16 RawData[32])
{
	u8 Channel;

	if (READDATA_DBG != 0)
		xil_printf("Waiting for EOS...\r\n");

	// Clear the Status
	XSysMon_GetStatus(InstancePtr);
	// Wait until the End of Sequence occurs
	while ((XSysMon_GetStatus(InstancePtr) & XSM_SR_EOS_MASK) != XSM_SR_EOS_MASK);

	if (READDATA_DBG != 0)
		xil_printf("Capturing XADC Data...\r\n");

	for (Channel=0; Channel<32; Channel++) {
		if (((1 << Channel) & XADC_CHANNELS) != 0) {
			if (READDATA_DBG != 0)
				xil_printf("Capturing Data for Channel %d\r\n", Channel);

			if(Source == 0){
				RawData[17] = XSysMon_GetAdcData(InstancePtr, 17) & INPUT_SCALE;
							on_time = 25000 + ((float)RawData[17] / (float)0xFFFF) * 100000;
				//			on_time = 500000 + ((float)RawData[17] / (float)0xFFFF) * 2000000;
							motor_duty_cycle = 6000000 + ((float)RawData[17] / (float)0xFFFF) * 14000000;
							changed_period = 2000000 + ((float)RawData[17] / (float)0xFFFF) * 400000;
							led_high_time = 0 + ((float)RawData[17] / (float)0xFFFF) * 20000000;
				//			printf("RawData: %x %d\r\n", RawData[17], 17);
							printf("on_time: %d\r\n", on_time);
							ResetTimer();
			}
			else if(Source == 1){
				RawData[25] = XSysMon_GetAdcData(InstancePtr, 25) & INPUT_SCALE;
				on_time = 25000 + ((float)RawData[25] / (float)0xFFFF) * 100000;
				//			on_time = 500000 + ((float)RawData[17] / (float)0xFFFF) * 2000000;
				motor_duty_cycle = 6000000 + ((float)RawData[25] / (float)0xFFFF) * 14000000;
				changed_period = 2000000 + ((float)RawData[25] / (float)0xFFFF) * 400000;
				led_high_time = 0 + ((float)RawData[25] / (float)0xFFFF) * 20000000;
				ResetTimer();
			}


		}
	}
	return XADC_CHANNELS; // return a high bit for each channel successfully read
}

void Xadc_Demo(XSysMon *InstancePtr, u32 ChannelSelect) {
//	printf("Inside xadc Demo\n");
	u16 Xadc_RawData[32];
	u32 ChannelValidVector;
	float Xadc_VoltageData;
	ChannelValidVector = Xadc_ReadData(InstancePtr, Xadc_RawData);
	if (Test_Bit(ChannelValidVector, ChannelSelect))
	{
		Xadc_VoltageData = Xadc_RawToVoltage(Xadc_RawData[ChannelSelect], ChannelSelect);
//		printf("Analog Input %s: %.3fV\r\n", Channel_Names[ChannelSelect], Xadc_VoltageData);
		if (Xadc_VoltageData > 0.5)
		{
//			printf("Voltage > 0.5\n");
		}
		else if (Xadc_VoltageData < -0.5)
		{
//			printf("Voltage is < -0.5\n");
		}
		else
		{
//			printf("Otherwise\n");
		}
	}
	else {
//		printf("Channel %d (%s) Not Available\r\n", (int)ChannelSelect, Channel_Names[ChannelSelect]);
	}
}

/******************************************************************************/
/**
*
* This is the entry function from the TestAppGen tool generated application
* which tests the interrupts when enabled in the GPIO
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a reference to the GPIO driver Instance
* @param	DeviceId is the XPAR_<GPIO_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_IP2INTC_IRPT_INTR
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
* @param	DataRead is the pointer where the data read from GPIO Input is
*		returned
*
* @return
*		- XST_SUCCESS if the Test is successful
*		- XST_FAILURE if the test is not successful
*
* @note		None.
*
******************************************************************************/
int GpioIntrExample(INTC *IntcInstancePtr, XGpio* InstancePtr, u16 DeviceId,
			u16 IntrId, u16 IntrMask, u32 *DataRead)
{
	int Status;

	/* Initialize the GPIO driver. If an error occurs then exit */
	Status = XGpio_Initialize(InstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = GpioSetupIntrSystem(IntcInstancePtr, InstancePtr, DeviceId,
					IntrId, IntrMask);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	GpioCheckForIntr(DataRead);

	return Status;
}

/******************************************************************************/
/**
*
* This function performs the GPIO set up for Interrupts
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a reference to the GPIO driver Instance
* @param	DeviceId is the XPAR_<GPIO_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_IP2INTC_IRPT_INTR
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
*
* @note		None.
*
******************************************************************************/
int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask)
{
	int Result;

	GlobalIntrMask = IntrMask;

#ifdef XPAR_INTC_0_DEVICE_ID

#ifndef TESTAPP_GEN
	/*
	 * Initialize the interrupt controller driver so that it's ready to use.
	 * specify the device ID that was generated in xparameters.h
	 */
	Result = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

	/* Hook up interrupt service routine */
	XIntc_Connect(IntcInstancePtr, IntrId,
		      (Xil_ExceptionHandler)GpioHandler, InstancePtr);

	/* Enable the interrupt vector at the interrupt controller */
	XIntc_Enable(IntcInstancePtr, IntrId);

#ifndef TESTAPP_GEN
	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor
	 */
	Result = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

#else /* !XPAR_INTC_0_DEVICE_ID */

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Result = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Result != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif /* TESTAPP_GEN */

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
					0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Result = XScuGic_Connect(IntcInstancePtr, IntrId,
				 (Xil_ExceptionHandler)GpioHandler, InstancePtr);
	if (Result != XST_SUCCESS) {
		return Result;
	}

	/* Enable the interrupt for the GPIO device.*/
	XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */

	/*
	 * Enable the GPIO channel interrupts so that push button can be
	 * detected and enable interrupts for the GPIO device
	 */
	XGpio_InterruptEnable(InstancePtr, IntrMask);
	XGpio_InterruptGlobalEnable(InstancePtr);

	/*
	 * Initialize the exception table and register the interrupt
	 * controller handler with the exception table
	 */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)INTC_HANDLER, IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This is the interrupt handler routine for the GPIO for this example.
*
* @param	CallbackRef is the Callback reference for the handler.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void GpioHandler(void *CallbackRef)
{
	XGpio *GpioPtr = (XGpio *)CallbackRef;

	int buttondata;
	IntrFlag = 1;

//	buttondata = XGpio_DiscreteRead(&Gpio, 1);
	buttondata = Xil_In32(XPAR_GPIO_1_BASEADDR);

	if(buttondata == 1)
	{
		LCD_command(1);
		resetSig = 1;
		//print("BTN0\r\n");
	}
	else if(buttondata == 2)
	{
		LCD_command(1);
		sourceSig = 1;
		//print("BTN1\r\n");
	}
	else if(buttondata == 4)
	{
		print("buttonData = 4\n");
		LCD_command(1);
		enaSig = 1;
		//print("BTN2\r\n");
	}
	else
	{
		;
	}

	delayMs(500);

	/* Clear the Interrupt */
	XGpio_InterruptClear(GpioPtr, GlobalIntrMask);

}

void GpioCheckForIntr(u32 *DataRead)
{
	u32 delay;

	IntrFlag = 0;
	delay = 0;

	while(!IntrFlag && (delay < INTR_DELAY)) {
		delay++;
	}

	if(enaSig == 1)
	{
		*DataRead = *DataRead + 1;
		if(*DataRead >= 2)
			*DataRead = 0;
		enaSig = 0;
	}
}

void GpioCheckForIntrLoop(u32 *DataRead, u32 *Source)
{
	u32 delay;

	IntrFlag = 0;
	delay = 0;

	while(!IntrFlag && (delay < INTR_DELAY)) {
		delay++;
	}

	if(enaSig == 1)
	{
		if(*DataRead == 0)
			*DataRead = 1;
		else
			*DataRead = 0;
//		*DataRead = *DataRead + 1;
//		*DataRead = not;
//		if(*DataRead >= 2)
//			*DataRead = 0;
		enaSig = 0;
	}

	if(sourceSig == 1)
	{
		*Source = *Source + 1;
		if(*Source >= 2)
			*Source = 0;
		sourceSig = 0;
	}
}

void LCD_init(void)
{
    delayMs(30);                /* initialization sequence */
    LCD_nibble_write(0x30, 0);
    delayMs(10);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
    delayMs(1);

    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0F);          /* turn on display, cursor blinking */
}

void LCD_nibble_write(unsigned char data, unsigned char control)
{
    data &= 0xF0;       /* clear lower nibble for control */
    control &= 0x0F;    /* clear upper nibble for data */
    Xil_Out8(XPAR_AXI_GPIO_0_BASEADDR, data | control); //PTD->PDOR = data | control;       /* RS = 0, R/W = 0 */
    Xil_Out8(XPAR_AXI_GPIO_0_BASEADDR, data | control | EN); //PTD->PDOR = data | control | EN;  /* pulse E */
    delayMs(0);
    Xil_Out8(XPAR_AXI_GPIO_0_BASEADDR, data); //PTD->PDOR = data;
    Xil_Out8(XPAR_AXI_GPIO_0_BASEADDR, 0); //PTD->PDOR = 0;
}

void LCD_command(unsigned char command)
{
    LCD_nibble_write(command & 0xF0, 0);   /* upper nibble first */
    LCD_nibble_write(command << 4, 0);     /* then lower nibble */

    if (command < 4)
        delayMs(4);         /* commands 1 and 2 need up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(unsigned char data)
{
    LCD_nibble_write(data & 0xF0, RS);    /* upper nibble first */
    LCD_nibble_write(data << 4, RS);      /* then lower nibble  */

    delayMs(1);
}


void delayMs(int n) {
	int microsecond = n * 1000; /* convert msec to usec */
	usleep(microsecond);		/* delay */
    //int i;
    //int j;
    //for(i = 0 ; i < n; i++)
        //for(j = 0 ; j < 7000; j++) { }
}
