//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);

void setupSpib(void);
void serialRXA(serial_t *s, char data);
void init_eQEPs(void);
void setEPWM6A(float controleffort);
void setEPWM6B(float controleffort);
void setDACA(float dacouta0);
void setDACB(float dacouta1);

float readEncLeft(void);
float readEncRight(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t spisend1 = 0;
uint16_t spisend2 = 0;
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
uint16_t rc1msb1;
uint16_t rc1lsb1;
uint16_t rc1msb2;
uint16_t rc1lsb2;
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
uint16_t ADCreading1;
uint16_t ADCreading2;
uint16_t adca0result = 0;
uint16_t adca1result = 0;
float ADCinvol1;
float ADCinvol2;
float accelx;
float accely;
float accelz;
float gyrox;
float gyroy;
float gyroz;
float LeftWheel;
float RightWheel;
float LeftWheel_1;
float RightWheel_1;
float Vref = 0.0; //                                          Vref
float VLeftK = 0.0;
float VRightK = 0.0;
float VLeftK_1 = 0.0;
float VRightK_1 = 0.0;
float vel_Left;
float vel_Right;
float ubal;
float uLeft;
float uRight;
float KPturn = 3.0;
float yk1 = 0.0;
float yk2 = 0.0;
float tilt_rate;
float WhlDiff;
float WhlDiff_1 = 0.0;
float vel_WhlDiff = 0.0;
float vel_WhlDiff_1 = 0.0;
float turn = 0.0;
float turnref = 0.0;                                                           //turn angle
float turnref_1 = 0.0;
float errorDiff = 0.0;
float errorDiff_1 = 0.0;
float intDiff = 0.0;
float intDiff_1 = 0.0;
float turnrate = 0.0;
float turnrate_1 = 0.0;
float FwdBackOffset = 0.0;
float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset  = 0;
float gyroy_offset  = 0;
float gyroz_offset  = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp;
int16_t doneCal = 0;
float tilt_value    = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value    = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001;        //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;



void setDACA(float dacouta0) {
    if (dacouta0 >  3.0) dacouta0 =  3.0;
    if (dacouta0 < 0.0) dacouta0 = 0.0;

    DacaRegs.DACVALS.bit.DACVALS = 4095.0 / 3.0 * dacouta0;  // perform scaling of 0-3 to 0-4095
}

void setDACB(float dacouta1) {
    if (dacouta1 >  3.0) dacouta1 =  3.0;
    if (dacouta1 < 0.0) dacouta1 = 0.0;

    DacbRegs.DACVALS.bit.DACVALS = 4095.0 / 3.0 * dacouta1;  // perform scaling of 0-3 to 0-4095
}

void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep
    EQep1Regs.QPOSCNT = 0;

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep2Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep
    EQep2Regs.QPOSCNT = 0;
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 18.7.
    return (raw*(2.0*PI/1496.0));
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 18.7.
    return (raw*(-2.0*PI/1496.0));
}


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();


    // with EPWM6A, EPWM6B
    EPwm6Regs.TBCTL.bit.CTRMODE = 0;
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm6Regs.TBCTL.bit.PHSEN = 0;
    EPwm6Regs.TBCTL.bit.CLKDIV = 0;
    EPwm6Regs.TBCTR = 0;
    EPwm6Regs.TBPRD = 2500;
    EPwm6Regs.CMPA.bit.CMPA = 1000;
    EPwm6Regs.CMPB.bit.CMPB = 1000;
    EPwm6Regs.AQCTLA.bit.CAU = 1;
    EPwm6Regs.AQCTLA.bit.ZRO = 2;
    EPwm6Regs.AQCTLB.bit.CBU = 1;
    EPwm6Regs.AQCTLB.bit.ZRO = 2;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;

    // LED1 is connected to GPIO22, change the Mux so that it is instead EPWM12A
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);

    // The right motor PWM input is connected to GPIO10, change the Mux so that it is EPWM6A
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);

    // The left motor PWM is connected to GPIO11, change the Mux so that it is EPWM6B
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);

    // The Launchpad RC low-pass filter circuit is connected to GPIO159, change the Mux so that it is EPWM8A.
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 1);

    //  disable the pull-up resistor when an I/O pin is set as a PWM pin for power consumption reasons
    EALLOW;  // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;
    GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;
    EDIS;


    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    //    // LED1 and PWM Pin
    //    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    //    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    //    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    // LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED9
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED10
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

    // LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

    // LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    // LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

    // LED14
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

    // LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

    // LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    // LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED20
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED21
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //PushButton 1
    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(122, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(123, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_INPUT, GPIO_PULLUP);

    // Part 1
    // command the ADCD peripheral to sample ADCIND0 and ADCIND1 every 1ms
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm5Regs.TBPRD = 50000;  // Set Period to 1ms sample.  Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    EDIS;


    // Part 2
    // setup ADCD so that it uses 2 of its 16 SOCs
    EALLOW;

    //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    //Select the channels to convert and end of conversion flag
    //Many statements commented out,  To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;

    // Part 3
    //  initialize for DACA and DACB output
    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;   //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  //use ADC VREF as reference voltage

    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;   //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;  //use ADC VREF as reference voltage
    EDIS;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    setupSpib();
    init_eQEPs();

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    //assign PieVectTable.ADCD1_INT to the memory address location of your ISR function
    PieVectTable.ADCA1_INT=&ADCA_ISR;

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 4000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    //    init_serial(&SerialC,115200,serialRXC);
    //    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; // SPIB_RX

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB_RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Enable SPIB_RX in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM



    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"tilt_value: %f, gyro_value: %f, LeftWheel: %f, RightWheel: %f \r\n", tilt_value, gyro_value, LeftWheel, RightWheel);
            UARTPrint = 0;
        }
    }
}

uint32_t numADCA1 = 0;
float xkn1[10] = {0};
float xkn2[10] = {0};
int n = 9;
float b[10]={   1.1982297073578186e-02,
                3.2593697188218529e-02,
                8.8809724362308426e-02,
                1.5903360855022139e-01,
                2.0758067282567344e-01,
                2.0758067282567344e-01,
                1.5903360855022139e-01,
                8.8809724362308426e-02,
                3.2593697188218529e-02,
                1.1982297073578186e-02};
__interrupt void ADCA_ISR (void)
{
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;

    // Here covert ADCIND0, ADCIND1 to volts
    xkn1[0] = 3.0 / 4095.0 * adca0result;
    xkn2[0] = 3.0 / 4095.0 * adca1result;

    int i;
    yk1 = 0.0;
    yk2 = 0.0;
    for (i=0; i<=n; i++) {
        yk1 += b[i] * xkn1[i];
        yk2 += b[i] * xkn2[i];
    }

    //Save past states before exiting from the function so that next sample they are the older state
    for(i=n; i>=1; i--) {
        xkn1[i] = xkn1[i-1];
        xkn2[i] = xkn2[i-1];

    }

    // Here write values to DAC channels
//    setDACA(yk1);
//    setDACB(yk2);

    // SPIB transmission
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));
    SpibRegs.SPITXBUF = 0; // Accel_x
    SpibRegs.SPITXBUF = 0; // Accel_y
    SpibRegs.SPITXBUF = 0; // Accel_z
    SpibRegs.SPITXBUF = 0; // Temp
    SpibRegs.SPITXBUF = 0; // Gyro_x
    SpibRegs.SPITXBUF = 0; // Gyro_x
    SpibRegs.SPITXBUF = 0; // Gyro_x


    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
    numADCA1++;
    if ((numADCA1 % 100) == 0) {
        UARTPrint = 1;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


uint16_t ADC1msb;
uint16_t ADC1lsb;
uint16_t ADC2msb;
uint16_t ADC2lsb;
int16_t x_acc_16;
int16_t y_acc_16;
int16_t z_acc_16;
int16_t x_gyr_16;
int16_t y_gyr_16;
int16_t z_gyr_16;
uint32_t numSPIcalls = 0;

int flag = 0;
float ekL;
float ekR;
float ekL_1;
float ekR_1;
float ukL;
float ukR;
float IkL;
float IkR;
float IkL_1;
float IkR_1;
float eturn;

__interrupt void SPIB_isr(void) {

    //    ADCre = SpibRegs.SPIRXBUF;
    //    ADC1msb = SpibRegs.SPIRXBUF; // Read first 8 bit value off RX FIFO. Probably is zero since no chip         **********
    //    ADC1lsb = SpibRegs.SPIRXBUF; // Read second 8 bit value off RX FIFO. Again probably zero
    //    ADC2msb = SpibRegs.SPIRXBUF; // Read first 8 bit value off RX FIFO. Probably is zero since no chip         **********
    //    ADC2lsb = SpibRegs.SPIRXBUF; // Read second 8 bit value off RX FIFO. Again probably zero
    //    ADCreading1 = (ADC1msb<<8) | ADC1lsb;
    //    ADCreading2 = (ADC2msb<<8) | ADC2lsb;
    //    ADCinvol1 = 3.3 * ADCreading1 / 1023.0;
    //    ADCinvol2 = 3.3 * ADCreading2 / 1023.0;
    //    GpioDataRegs.GPASET.bit.GPIO2 = 1; // Set GPIO 2 to end Slave Select. Now to Scope. Later to deselect DAN777
    //    // Later when actually communicating with the DAN777 do something with the data. Now do nothing.
    //    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    //    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    //    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt


    temp = SpibRegs.SPIRXBUF;
    x_acc_16 = SpibRegs.SPIRXBUF;
    y_acc_16 = SpibRegs.SPIRXBUF;
    z_acc_16 = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    x_gyr_16 = SpibRegs.SPIRXBUF;
    y_gyr_16 = SpibRegs.SPIRXBUF;
    z_gyr_16 = SpibRegs.SPIRXBUF;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    accelx = 4.0 * x_acc_16 /32767.0;
    accely = 4.0 * y_acc_16 /32767.0;
    accelz = 4.0 * z_acc_16 /32767.0;
    gyrox = 250.0 * x_gyr_16 / 32767.0;
    gyroy = 250.0 * y_gyr_16 / 32767.0;
    gyroz = 250.0 * z_gyr_16 / 32767.0;

    if (accelx>4.0) {accelx = 4.0;}
    else if (accelx<-4.0) {accelx = -4.0;}
    if (accely>4.0) {accely = 4.0;}
    else if (accely<-4.0) {accely = -4.0;}
    if (accelz>4.0) {accelz = 4.0;}
    else if (accelz<-4.0) {accelz = -4.0;}

    if (gyrox>250.0) {gyrox = 250.0;}
    else if (gyrox<-250.0) {gyrox = -250.0;}
    if (gyroy>250.0) {gyroy = 250.0;}
    else if (gyroy<-250.0) {gyroy = -250.0;}
    if (gyroz>250.0) {gyroz = 250.0;}
    else if (gyroz<-250.0) {gyroz = -250.0;}

    LeftWheel = readEncLeft();
    RightWheel = readEncRight();

//    setEPWM6A(uLeft);
//    setEPWM6B(-uRight);

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
      if(calibration_state == 0){
            calibration_count++;
            if (calibration_count == 2000) {
                calibration_state = 1;
                calibration_count = 0;
            }
        } else if(calibration_state == 1){
            accelx_offset+=accelx;
            accely_offset+=accely;
            accelz_offset+=accelz;
            gyrox_offset+=gyrox;
            gyroy_offset+=gyroy;
            gyroz_offset+=gyroz;
            calibration_count++;
            if (calibration_count == 2000) {
                calibration_state = 2;
                accelx_offset/=2000.0;
                accely_offset/=2000.0;
                accelz_offset/=2000.0;
                gyrox_offset/=2000.0;
                gyroy_offset/=2000.0;
                gyroz_offset/=2000.0;
                calibration_count = 0;
                doneCal = 1;
            }

        } else if(calibration_state == 2){

            accelx -=(accelx_offset);
            accely -=(accely_offset);
            accelz -=(accelz_offset-accelzBalancePoint);
            gyrox -= gyrox_offset;
            gyroy -= gyroy_offset;
            gyroz -= gyroz_offset;

            /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
            float tiltrate = (gyrox*PI)/180.0; // rad/s
            float pred_tilt, z, y, S;

            tilt_rate = tiltrate;

            // Prediction Step
            pred_tilt = kalman_tilt + T*tiltrate;
            pred_P = kalman_P + Q;

            // Update Step
            z = -accelz;
            y = z - pred_tilt;
            S = pred_P + R;
            kalman_K = pred_P/S;
            kalman_tilt = pred_tilt + kalman_K*y;
            kalman_P = (1 - kalman_K)*pred_P;

            SpibNumCalls++;
            // Kalman Filter used
            tilt_array[SpibNumCalls] = kalman_tilt;
            gyro_array[SpibNumCalls] = tiltrate;
            LeftWheelArray[SpibNumCalls] = readEncLeft();
            RightWheelArray[SpibNumCalls] = readEncRight();

            if (SpibNumCalls >= 3) {  // should never be greater than 3
                tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
                gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
                LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
                RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
                SpibNumCalls = -1;

                PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cau1qA    se the interrupt for the SWI
            }
        }

        timecount++;

        if((timecount%200) == 0)
        {
            if(doneCal == 0) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Blink Blue LED while calibrating
            }
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Always Block Red LED

            UARTPrint = 1;  // Tell While loop to print
        }

        SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
        SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}

int SWIflag = 0;
float K1 = -30.0;
float K2 = -2.8;
float K3 = -1.0;

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {


    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......
    VLeftK = (LeftWheel - LeftWheel_1)/ 0.004;
    VRightK = (RightWheel - RightWheel_1) / 0.004;
    vel_Left = 0.6 * VLeftK_1 + 0.4 * VLeftK;
    vel_Right = 0.6 * VRightK_1 + 0.4 * VRightK;

    turnref = turnref_1 + 0.002*(turnrate + turnrate_1);
    WhlDiff = LeftWheel - RightWheel;
    vel_WhlDiff = 1.0/3.0 * vel_WhlDiff_1 + 2.0/3.0 * (WhlDiff - WhlDiff_1)/ 0.004;
    WhlDiff_1 = WhlDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    errorDiff = turnref - WhlDiff;
    intDiff = intDiff_1 + 0.002*(errorDiff + errorDiff_1);
    turn = Kp * errorDiff + Ki * intDiff - Kd * vel_WhlDiff;
    if (fabs(turn)>3.0) {
        intDiff = intDiff_1;
    }
    if (turn>4.0) { turn = 4.0;}
    if (turn<-4.0) { turn = -4.0;}

    errorDiff_1 = errorDiff;
    intDiff_1 = intDiff;

    ubal = -K1 * tilt_value + -K2 * gyro_value + -K3 * (vel_Left + vel_Right)/2.0;
    uLeft = ubal/2.0 - turn + FwdBackOffset;
    uRight = ubal/2.0 + turn + FwdBackOffset;

    setEPWM6A(uLeft);
    setEPWM6B(-uRight);


    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    VLeftK_1 = vel_Left;
    VRightK_1 = vel_Right;
    turnref_1 = turnref;
    turnrate_1 = turnrate;




    numSWIcalls++;

    // Blink a number of LEDS
    //    GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;
    //    GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    //    GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;
    //    GpioDataRegs.GPETOGGLE.bit.GPIO157 = 1;
    //    GpioDataRegs.GPETOGGLE.bit.GPIO158 = 1;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{

    CpuTimer0.InterruptCount++;

    //    numTimer0calls++;
    //    if ((numTimer0calls%10) == 0) {
    //        UARTPrint = 1;
    //    }

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }


    // Blink LaunchPad Red LED
//    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    // Blink a number of LEDS
    //    GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO25 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    // Blink a number of LEDS
    //    GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    //    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;
    //    GpioDataRegs.GPCTOGGLE.bit.GPIO67 = 1;
    //    GpioDataRegs.GPCTOGGLE.bit.GPIO94 = 1;
    //    GpioDataRegs.GPCTOGGLE.bit.GPIO95 = 1;
    //    GpioDataRegs.GPDTOGGLE.bit.GPIO97 = 1;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
//    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    // Blink a number of LEDS
    //    GpioDataRegs.GPDTOGGLE.bit.GPIO111 = 1;
    //    GpioDataRegs.GPETOGGLE.bit.GPIO130 = 1;
    //    GpioDataRegs.GPETOGGLE.bit.GPIO131 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1;
    //    GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
    CpuTimer2.InterruptCount++;
    //  if ((CpuTimer2.InterruptCount % 50) == 0) {
    //      UARTPrint = 1;
    //  }
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'a') {
        turnrate = turnrate - 0.2;
    } else if (data == 'd') {
        turnrate = turnrate + 0.2;
    } else if (data == 'w') {
        FwdBackOffset = FwdBackOffset - 0.2;
    } else if (data == 's') {
        FwdBackOffset = FwdBackOffset + 0.2;
    } else {
        turnrate = 0;
        FwdBackOffset = 0;
    }
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{

    // Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3.  Change so that 16 bits are transmitted each TX FIFO write and change the delay in between each transfer to 0.
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0); // Set as GPIO2 and used as DAN777 SS
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO2 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO2 = 1; //Initially Set GPIO2/SS High so DAN777 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set prequalifier for SPI PINS
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // The prequalifier eliminates short noise spikes
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // by making sure the serial pin stays low for 3 clock periods.
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN777 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 8 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; //Set delay between transmits to 16 spi clocks. Needed by DAN777 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 2; //Interrupt Level to 2 words or more received into FIFO causes interrupt
    //-----------------------------------------------------------------------------------------------------------------

    // Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers
    SpibRegs.SPITXBUF = 0x1300; // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0013; // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0200; // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0806; // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0000; // To address 00x1E write 0x00
    // To address 00x1F write 0x00


    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF; //1
    temp = SpibRegs.SPIRXBUF; //2
    temp = SpibRegs.SPIRXBUF; //3
    temp = SpibRegs.SPIRXBUF; //4
    temp = SpibRegs.SPIRXBUF; //5
    temp = SpibRegs.SPIRXBUF; //6
    temp = SpibRegs.SPIRXBUF; //7
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers
    SpibRegs.SPITXBUF = 0x2300; // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x408C; // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x0288; // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0C0A; // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF; //1
    temp = SpibRegs.SPIRXBUF; //2
    temp = SpibRegs.SPIRXBUF; //3
    temp = SpibRegs.SPIRXBUF; //4
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001);  // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001);  // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001);  // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020);  // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071);  // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

// This function is called to control one of the motors according to the giving controleffort.
// The negative or positive of the value of controleffort determine the direction of spin of motor.
// The absolute value of controleffort determine the speed of spin of the motor.
// The input value of controleffort is controled in the range of [-10,10]
void setEPWM6A(float controleffort) {
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort >= 0) {
        GpioDataRegs.GPASET.bit.GPIO29 = 1;
    }
    else {
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
    }

    EPwm6Regs.CMPA.bit.CMPA = fabs(controleffort) / 10.0 * EPwm6Regs.TBPRD;

}

// This function is called to control the other motor
// The motor will spin in an opposite direction as the former but in the same speed.
void setEPWM6B(float controleffort) {
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort >= 0) {
        GpioDataRegs.GPBSET.bit.GPIO32 = 1;
    }
    else {
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    }

    EPwm6Regs.CMPB.bit.CMPB = fabs(controleffort) / 10.0 * EPwm6Regs.TBPRD;
}
