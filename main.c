/******************************************************************************************************************/
/* main.c                                                                                                         */
/*                                                                                                                */
/*	Cliente:  CONTEMP																							  */
/*	Projeto:  NOVA LINHA PIC24  																				  */
/*	Responsavel: DANIEL FERREIRA PEIXOTO																		  */
/*	Firmware: Teste I2C MCP3425                                                                                   */
/*  Hardware compativel: Placa Padrao                                                                             */
/*	C.I compativel: MCP3425 (ADC 16bits)                                                                          */
/*                                                                                                                */
/*		Versao Atual: V1.0                                                                                        */
/*                                                                                                                */
/* 			IDE: MPLABX V3.55                                                                                     */
/* 			Compilador: Microchip XC16 V1.31                                                                      */
/* 			Microcontrolador: PIC24FJ128GC006                                                                     */
/*                                                                                                                */
/* 			Historico de versao:                                                                                  */
/*			V1.0 - (Release) 26/07/17 - Daniel F. Peixoto                                                         */
/*                                                                                                                */
/*                                                                                                                */
/******************************************************************************************************************/

/****************************************************** Pragma ****************************************************/
// <editor-fold defaultstate="collapsed" desc="CONFIGURATION BITS">
// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable bit->DSBOR Disabled
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable->DSWDT Disabled
#pragma config DSSWEN = OFF             // DSEN Bit Enable->Deep Sleep operation is always disabled
#pragma config RTCBAT = OFF             // RTC Battery Operation Enable->RTC operation is discontinued in VBAT
#pragma config PLLDIV = DIV2            // PLL Input Prescaler Select bits->Oscillator divided by 2 (8 MHz input)
#pragma config I2C2SEL = SEC            // I2C2 is multiplexed to SDA2/RF4 and SCL2/RF5
#pragma config IOL1WAY = OFF            // PPS IOLOCK Set Only Once Enable bit->The IOLOCK bit can be set and cleared using the unlock sequence

// CONFIG3
#pragma config WPFP = WPFP127           // Write Protection Flash Page Segment Boundary->Page 127 (0x1FC00)
#pragma config SOSCSEL = OFF            // SOSC Selection bits->SOSC circuit selected
#pragma config WDTWIN = PS50_0          // Window Mode Watchdog Timer Window Width Select->Watch Dog Timer Window Width is 50 percent
#pragma config BOREN = OFF              // Brown-out Reset Enable->DSBOR Disabled
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable->Disabled
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select->Disabled
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory

// CONFIG2
#pragma config POSCMD = XT              // Primary Oscillator Select->XT Oscillator Enabled
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits->WDT uses LPRC
#pragma config OSCIOFCN = OFF           // OSCO Pin Configuration->OSCO/CLKO/RC15 functions as port I/O (RC15)
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC = PRIPLL           // Initial Oscillator Select->Primary Oscillator with PLL module (XTPLL,HSPLL, ECPLL)

#pragma config ALTADREF = AVREF_RB      // External 12-Bit A/D Reference Location Select bit->AVREF+/AVREF- are mapped to RB0/RB1
#pragma config ALTCVREF = CVREF_RB      // External Comparator Reference Location Select bit->CVREF+/CVREF- are mapped to RB0/RB1
#pragma config WDTCMX = WDTCLK          // WDT Clock Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config IESO = OFF               // Internal External Switchover->Disabled

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select->1:32768
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF             // Windowed WDT Disable->Standard Watchdog Timer
#pragma config FWDTEN = WDT_DIS         // Watchdog Timer Enable->WDT disabled in hardware; SWDTEN bit disabled
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC1/PGED1
#pragma config LPCFG = OFF              // Low power regulator control->Disabled - regardless of RETEN
#pragma config GWRP = OFF               // General Segment Write Protect->Disabled
#pragma config GCP = OFF                // General Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF             // JTAG Port Enable->Disabled
// </editor-fold>
/******************************************************************************************************************/

/***************************************************** Include ****************************************************/
// <editor-fold defaultstate="collapsed" desc="INCLUDE">
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "I2C1.h"
#include "I2C2.h"
#include "MCP3425.h"
#include "p24FJ128GC006.h"
// </editor-fold>
/******************************************************************************************************************/

/****************************************************** Define ****************************************************/

/******************************************************************************************************************/

/***************************************************** Constant ***************************************************/

/******************************************************************************************************************/

/*********************************************** Extern Global Variable *******************************************/
extern MCP3425_type MCP3425;
/******************************************************************************************************************/

/*************************************************** Global Variable **********************************************/

/******************************************************************************************************************/

/************************************************* Function Prototype *********************************************/
// <editor-fold defaultstate="collapsed" desc="FUNCTION PROTOTYPE">
void SYSTEM_Initialize(void);
void OSCILLATOR_Initialize(void);
void PIN_MANAGER_Initialize(void);
void PERIPHERAL_Initialize(void);
void INTERRUPT_Initialize (void);
// </editor-fold>
/******************************************************************************************************************/
/******************************************************************************************************************/

/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************** Main ****************************************************/
int main(void)
{    
        // initialize the device
        SYSTEM_Initialize();   
        
        

        ADC_Initialize(MCP3425_A0, MCP3425_I2C1, MCP3425_DEVICE_CODE, CONTINUOUS, GAIN_1_VV, SAMPLE_RATE_15SPS_16BITS, &MCP3425);
      
        //------------------------------------------------------ Read ------------------------------------------------------// 
        // <editor-fold defaultstate="collapsed" desc="Read"> 
        // READ ADC REGISTER
        Read_ADC_REGISTER(&MCP3425);
        
        // READ ADC 
        Read_ADC(&MCP3425);
        // </editor-fold>
        //------------------------------------------------------------------------------------------------------------------//    
       
        //--------------------------------------------------- Update->Write ------------------------------------------------//
        // <editor-fold defaultstate="collapsed" desc="Write">
        // WRITE ADC REGISTER
        Update_ADC_REGISTER (CONTINUOUS, GAIN_4_VV, SAMPLE_RATE_15SPS_16BITS, &MCP3425);
        Write_ADC_REGISTER (&MCP3425);
        // </editor-fold>
        //------------------------------------------------------------------------------------------------------------------//
        
        
        
        // READ ADC REGISTER
        Read_ADC_REGISTER(&MCP3425);
            
        while (1)
        {
            Nop();  
        }
     
        return 0;
}


/******************************************************************************************************************/


/*************************************************** Configure MCU ************************************************/
void SYSTEM_Initialize(void)
{
        OSCILLATOR_Initialize();
        PIN_MANAGER_Initialize();       
        PERIPHERAL_Initialize();
        
        INTERRUPT_Initialize();
}

void OSCILLATOR_Initialize(void)
{   
        // WDTO disabled; TRAPR disabled; SLEEP disabled; BOR disabled; DPSLP disabled; CM disabled; SWR disabled; SWDTEN disabled; EXTR disabled; POR disabled; SBOREN disabled; IDLE disabled; IOPUWR disabled; VREGS disabled; 
        RCON = 0x0000;
}
/******************************************************************************************************************/

/**************************************************** Configure IO ************************************************/
void PIN_MANAGER_Initialize(void)
{
        /****************************************************************************
         * Setting the Output Latch SFR(s)
         ***************************************************************************/
        LATB = 0x0000;
        LATC = 0x0000;
        LATD = 0x0000;
        LATE = 0x0000;
        LATF = 0x0000;
        LATG = 0x0000;

        /****************************************************************************
         * Setting the GPIO Direction SFR(s)
         ***************************************************************************/
        TRISB = 0x0000;
        TRISC = 0x0000; 
        TRISD = 0x0000;                                                                 
        TRISE = 0x0000;
        TRISF = 0X0000;                                                                 
        TRISG = 0x0000;

        /****************************************************************************
         * Setting the Weak Pull Up and Weak Pull Down SFR(s)
         ***************************************************************************/
        CNPD1 = 0x0000;
        CNPD2 = 0x0000;
        CNPD3 = 0x0000;
        CNPD4 = 0x0000;
        CNPD5 = 0x0000;
        CNPD6 = 0x0000;
        CNPU1 = 0x0000;
        CNPU2 = 0x0000;
        CNPU3 = 0x0000;
        CNPU4 = 0x0000;
        CNPU5 = 0x0000;
        CNPU6 = 0x0000;

        /****************************************************************************
         * Setting the Open Drain SFR(s)
         ***************************************************************************/
        ODCB = 0x0000;
        ODCC = 0x0000;
        ODCD = 0x0000;
        ODCE = 0x0000;
        ODCF = 0x0000;
        ODCG = 0x0000;
        
        /****************************************************************************
         * Setting the Analog/Digital Configuration SFR(s)
         ***************************************************************************/      
        ANSB = 0x0000;
        ANSD = 0x0000;
        ANSE = 0x0000;
        ANSF = 0x0000;
        ANSG = 0x0000;  
}
/******************************************************************************************************************/

/************************************************ Configure PERIPHERAL ********************************************/
void PERIPHERAL_Initialize(void)
{              
        //SDADC1 
        SD1CON1bits.SDON = 0;

        //TIMER
        T1CONbits.TON = false;
        T2CONbits.TON = false;
        T3CONbits.TON = false;
        T4CONbits.TON = false;
        T5CONbits.TON = false;

        //OC 
        OC1CON1bits.OCM = 0x000;
        OC2CON1bits.OCM = 0x000;
        OC3CON1bits.OCM = 0x000;
        OC4CON1bits.OCM = 0x000;
        OC5CON1bits.OCM = 0x000;          
        OC6CON1bits.OCM = 0x000;
        OC7CON1bits.OCM = 0x000;
        OC8CON1bits.OCM = 0x000;
        OC9CON1bits.OCM = 0x000;
    
        //OPA 
        AMP1CONbits.AMPEN = false;
        AMP2CONbits.AMPEN = false; 

        //PADC
        ADCON1bits.ADON = false;
        
        //IC
        IC1CON1bits.ICM = 0x000;
        IC2CON1bits.ICM = 0x000;
        IC3CON1bits.ICM = 0x000;
        IC4CON1bits.ICM = 0x000;
        IC5CON1bits.ICM = 0x000;
        IC6CON1bits.ICM = 0x000;
        IC7CON1bits.ICM = 0x000;
        IC8CON1bits.ICM = 0x000;
        IC9CON1bits.ICM = 0x000;
             
        //CMP
        CM1CONbits.CON = false;
        CM2CONbits.CON = false;
        CM3CONbits.CON = false;
        
        //DMA
        DMACONbits.DMAEN = false;
        
        //CTMU
        CTMUCON1bits.CTMUEN = false;
        
        //CVR
        CVRCONbits.CVREN = false;
        
        //HLVD
        HLVDCONbits.HLVDEN = false;   

        //RTCC
        RCFGCALbits.RTCEN = false;
        
        //MODULATOR
        MDCONbits.MDEN = false;  
      
        //DAC
        DAC1CONbits.DACEN = false;
        DAC2CONbits.DACEN = false;
        
        //EPMP
        PMCON1bits.PMPEN = false;
            
        //LCD
        LCDCONbits.LCDEN = false;
        
        //USB
        U1PWRCbits.USBPWR = false;
        U1CONbits.USBEN = false;
        
        //UART
        U1MODEbits.UARTEN = false;
        U2MODEbits.UARTEN = false;
        U3MODEbits.UARTEN = false;
        U4MODEbits.UARTEN = false;
        
        //SPI
        SPI1STATbits.SPIEN = false;
        SPI2STATbits.SPIEN = false;
              
        //I2C
        I2C1CONbits.I2CEN = false;
        I2C2CONbits.I2CEN = false;
        
        
        //I2C      
        I2C1_Initialize();  
        //I2C2_Initialize();
}
/******************************************************************************************************************/

/************************************************* Configure INTERRUPT ********************************************/
void INTERRUPT_Initialize (void)
{

}
/******************************************************************************************************************/
/******************************************************************************************************************/


/**************************************************** INTERRUPTION ************************************************/
/******************************************************************************************************************/
/******************************************** High priority interrupt vector **************************************/

/******************************************************************************************************************/

/********************************************* Low priority interrupt vector **************************************/

/******************************************************************************************************************/

/******************************************** High priority interrupt routine *************************************/

/******************************************************************************************************************/

/******************************************** Low priority interrupt routine **************************************/

/******************************************************************************************************************/
/******************************************************************************************************************/


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/  