/*Calibrates ADE9000*/
/*The calibration inputs are stored in the ADE9000CalibrationInputs.h file. The phase and parameter being calibrated is input through the serial console*/
/*Calbration constants are computed and stored in the EEPROM */
/*Caibration should be done with the end application settings. If any  parameters(GAIN,High pass corner,Integrator settings) are changed, the device should be recalibrated*/
/*This application assumues the PGA_GAIN among all current channels is same.Also, the PGA_GAIN among all voltage channels should be same*/

#include <SPI.h>
#include <ADE9000RegMap.h>
#include <ADE9000API.h>
#include <math.h>
#include <Wire.h>

/*Function declerations*/
void getPGA_gain();
void getIRMS_constant();
void ADE9000_current_gain_calibration();
void ADE9000_current_measurement();
void ADE9000_deep_sleep();
void ADE9000_deep_wake();
void timer0_irq();

/************************************************************************************************ 
 * CUR_CHAN_USED represents the amount of current channels used for calibration AND measurement *
 * if a sensor is demounted from a channel and the program is still running:                    *
 * than change the CUR_CHAN_USED value and restart the calibration values.                      *
 ************************************************************************************************/
#define CUR_CHAN_USED 3 //1: channel A, 2: channel A and B, 3: channel A, B and C, 4: Channel A, B, C and N
#define IGAIN_CAL_REG_SIZE 4
int32_t xIgain_registers[IGAIN_CAL_REG_SIZE];   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
int32_t xIgain_register_address[IGAIN_CAL_REG_SIZE]=
         {ADDR_AIGAIN, ADDR_BIGAIN, ADDR_CIGAIN, ADDR_NIGAIN};   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
float xIrms_registers[IGAIN_CAL_REG_SIZE];
int32_t xIrms_registers_address[IGAIN_CAL_REG_SIZE]= {ADDR_AIRMS, ADDR_BIRMS, ADDR_CIRMS, ADDR_NIRMS};


//Global variables
int8_t calCurrentPGA_gain = 0;
float iRms_conversionConstant = 0.0;
int8_t timer0_flag = LOW;

//Define and variable for activating serial debugger
#define ENABLED 1
#define DISABLED 0
char SERIAL_DEBUGGER = ENABLED; //serial debugger text, to disable, change to DISABLED

//Define for calibration
//CHANGE THIS WHEN YOU CHANGE THE CALIBRATION CONSTANTS SUCH AS:
//I nominal of the current source
//current transfer function of the sensors
#define I_NOMINAL 10 //Inominal value that is used for calibration
#define CURRENT_TRANSFER_FUNCTION 0.0000225 //current transfer function

ADE9000Class ade9000;
#define SPI_SPEED 5000000
#define CS_PIN 1
#define PM_1 4
#define IRQ_1 3

void setup() 
{  
  pinMode(IRQ_1, INPUT);    //IRQ1 geeft aan wanneer de start up sequence voltooid is
  pinMode(PM_1, OUTPUT);    //Set PM1 select pin as output 
  digitalWrite(PM_1, LOW);   //Set PM1 select pin high 
  delay(1000); //goedkope manier om de resetdone interrupt niet te gebruiken. Deze geeft aan als de ADE9000 opgestart is.
  Serial.begin(115200);
  ade9000.SPI_Init(SPI_SPEED, CS_PIN);
  ade9000.SetupADE9000();       /*Setup ADE9000. The setup parameters such as 
                                PGA GAIN should not be changed after calibration. Recalibrate if any configuration affecting the digital datapath changes.*/
  
  delay(1000); //zonder deze delay een deel van de serial writes van getPGA_gain en ADE9000_calibration niet zichtbaar.
  
  getPGA_gain();
  getIRMS_constant();
  ADE9000_current_gain_calibration();
  //ADE9000_deep_sleep();
  
  Serial.println("finished setup of the ADE9000");
  Serial.println("starting the measurements");
  Serial.println();
}

void loop()
{
  //als timer interrupt geactiveerd wordt, wordt een vlaggetje hoog
  timer0_irq(); //mag weg als de timer interrupt gemaakt is
  if (timer0_flag == HIGH)
  {
    timer0_flag = LOW; //reset the timer flag
    ADE9000_deep_wake(); //activate the normal mode(PSM0) of the ADE9000 and init the SPI conenction  
    ADE9000_current_measurement(); //measure the used current channels
    ADE9000_deep_sleep(); //activate the deepsleep mode(PSM3) of the ADE9000 and make the chip select low
  }  
}

//delay vervangen met de echte interrupt routine
void timer0_irq()
{
  delay(1000);
  timer0_flag = HIGH;
}

/*****************************************************
 * Activates the normal mode of the ADE9000
 * Activates and initialise the SPI connection 
 * Setup all registers of the ADE9000
 *****************************************************/
//haalt de ADE9000 uit diepe slaap en activeert alles en re init alles
void ADE9000_deep_wake()
{
  digitalWrite(PM_1, LOW); //activeer PSM1 normal power mode
  
  Serial.println(" Start up sequence is active");
  while (digitalRead(IRQ_1) == HIGH);
  Serial.println(" Start up sequence is finished, initialising SPI");
  delay(1000);
  
  //initialising and starting the SPI conenction
  ade9000.SPI_Init(SPI_SPEED, CS_PIN); //digitalWrite(CS_PIN, HIGH);
  Serial.println(" initialising SPI finished");
  Serial.println();
  
  //setup ADE9000 with all parameters such as PGA gain
  ade9000.SetupADE9000();
  delay(1000); //DIT WERKT OOK ZONDER DELAY, maar de delay in de setup nadat de class wordt geladen is wel nodig, om die reden is er voor de zekerheid hier ook een delay geplaatst.
  
  //Set the xIGAIN registers of the ADE9000 with the igain calibration values stored in the flash memory
  for(int i = 0; i < CUR_CHAN_USED; i++)
  {
    ade9000.SPI_Write_32(xIgain_register_address[i], xIgain_registers[i]);
  }
  delay(1000); //kan weg?!
}

//brengt de ADE9000 in een diepe slaap en deactiveert alles
void ADE9000_deep_sleep()
{
  //afschakelen van de SPI
  //digitalWrite(CS_PIN, LOW);   //deactivate the ADE9000 spi conenction (by deselecting CS_PIN)
  
  //PM_1 hoog maken
  digitalWrite(PM_1, HIGH); //activating PSM3 deep sleep mode 2uA power consumption

  //ADE9000 is nu uit
}

/******************************************************************
 * meten van de stroom kanalen                                    *
 * CUR_CHAN_USED bepaald hoeveel en welke kanalen gemeten worden  *
 ******************************************************************/
void ADE9000_current_measurement()
{
  float temp;
  for (int i = 0; i < CUR_CHAN_USED; i++)
  {
    xIrms_registers[i] = ade9000.SPI_Read_32(xIrms_registers_address[i]) * iRms_conversionConstant;
    Serial.print(" Measured current, current channel ");
    Serial.print(i+1); 
    Serial.print(" : ");
    Serial.print(xIrms_registers[i]);
    Serial.println(" (Arms)");
  }
  Serial.println();
}

/********************************************************************
 * Gets the PGA value that is initialised in the ADE9000API.h file  *
 * This value is stored in a global variable called pgaGainRegister *
 ********************************************************************/
void getPGA_gain()
{
  int16_t pgaGainRegister;
  int16_t temp;  
  pgaGainRegister = ade9000.SPI_Read_16(ADDR_PGA_GAIN);  //Ensure PGA_GAIN is set correctly in SetupADE9000 function.
  //Serial.print("PGA Gain Register is: ");
  //Serial.println(pgaGainRegister,HEX);
  temp =    pgaGainRegister & (0x0003);  //extract gain of current channel
  if (temp == 0)  // 00-->Gain 1: 01-->Gain 2: 10/11-->Gain 4
  {
    calCurrentPGA_gain = 1; 
  }
  else
  {
    if(temp == 1) calCurrentPGA_gain = 2;  
    else calCurrentPGA_gain = 4;  
  }
  Serial.print(" PGA gain for all used current channels: ");
  Serial.println(calCurrentPGA_gain);
}
/******************************************************************************
 * Calculates the conversion constant to convert the xIRMS to physical values *
 * The constant is set in the iRms_conversionConstant global variable         *
 ******************************************************************************/
void getIRMS_constant()
{
  iRms_conversionConstant = 1 / (CURRENT_TRANSFER_FUNCTION * calCurrentPGA_gain * sqrt(2) * ADE9000_RMS_FULL_SCALE_CODES);
  Serial.print(" Conversion constant: ");
  Serial.print(iRms_conversionConstant * 1000000);
  Serial.println(" uArms/code");
}

/************************** BELANGRIJK ****************************
 * Alle kanalen dienen tegelijkertijd worden gecalibreerd         *
 * Alle kanalen dienen dezelfde stroomsensoren te gebruiken       *
 * Alle kanalen dienen dezelfde nominale ingangstroom te krijgen  *
 * Alle kanalen dienen dezelfde PGA versterkingsfactor te hebben  *
 * Nominale ingangstroom voor calibratie moet boven de 1Arms zijn *
 ******************************************************************/
/****************************** xIGAIN calibration ***************************
 * Deze functie is bedoeld om de xIGAIN in te stellen                        *
 * Vergelijk de actuele xIRMS waarde met de 0.5Arms iNom threshold           *
 * De gain wordt berekend door: verwachte xIRMS / actuele xIRMS              *
 * De xIGAIN wordt ingesteld door: (gain - 1) * 2^27                         *
 * Als de xIGAIN registers is ingesteld is de xIRMS registers gecalibreerd   *
 *****************************************************************************/
/******************************  Verwachte xIRMS ****************************************************************************************************
 * Verwachte xIRMS wordt berekend door de volgende afgeleide formule:                                                                               *
 * Verwachte xIRMS = Nominale stroom * stroom transfer functie * PGA versterkingsfactor * wortel(2) * full scale RMS code van de ADC                *
 * De nominale stroom is een constante en is genoteerd als define --> I_NOMINAL (in Arms)                                                           *
 * De stroom transfer functie is bij een Rogowski spoel bekend, bij een stroomtrafo moet deze berekend worden                                       *
 * De stroom transfer functie is een constante en wordt genoteerd als define --> CURRENT_TRANSFER_FUNCTION (V per A)                                *
 * De PGA versterkingsfactor wordt ingesteld in ADE9000API.h --> ADE9000_PGA_GAIN                                                                   *
 * De ingestelde PGA versterkingsfactor moet voor ALLE stroomkanalen HET ZELFDE zijn!                                                               *
 * De ingestelde PGA versterkingsfactor wordt door de functie getPGA_gain opgehaald en geplaatst in de globale variabele --> calCurrentPGA_gain     *
 * De full scale RMS code van de ADC is voor de xIRMS registers gelijk aan 52702092                                                                 *
 * De full scale RMS code van de ADC is ingesteld in ADE9000API.h --> ADE9000_RMS_FULL_SCALE_CODES                                                  *
 ****************************************************************************************************************************************************/
/******************************   Actuele xIRMS *************************************************
 * Er moet een nominale stroom worden aangesloten op de te kalibreren kanalen                   *
 * De nominale stroom moet voor ALLE stroomkanalen HET ZELFDE zijn                              *
 * De waarde van deze nominale stroom moet worden genoteerd als define --> I_NOMINAL (in Arms)  *
 * Door xIGAIN register te nullen wordt elke vorm van calibratie verwijderd                     *
 * Nu kunnen de xIRMS register uitgelezen worden                                                *
 * Actuele xIRMS = xIRMS register waarde                                                        *
 ************************************************************************************************/
void ADE9000_current_gain_calibration()
{  
  char serialReadData; //variable for the serial input
  Serial.println(" Starting calibration process. Select one of the following {Start (Y/y) OR Abort(Q/q)}:");
  while (Serial.read() >= 0);  //Flush any extra characters
  while (!Serial.available());  //wait for serial data to be available
  serialReadData = Serial.read();
  
  if (serialReadData == 'Y' || serialReadData == 'y')
  {
    int32_t expectedIRMS = 0;
    int32_t actualIRMS = 0;
    int32_t iNomThresh = 0;
    float temp = 0.0;
    const float minimal_iNom = 0.5; //this is threshold current in Arms. If the iNominal is below 0.5Arms, there won't be a calibration of that channel

    //nulling all xIGAIN registers
    Serial.print(" Resetting xIGAIN registers. ");
    for (int i=0; i < CUR_CHAN_USED; i++) ade9000.SPI_Write_32(xIgain_register_address[i], 0x00000000);
    delay(500);
    Serial.println("Done.");
    Serial.println();
  
    //Calculating the expected xIRMS
    temp = I_NOMINAL * CURRENT_TRANSFER_FUNCTION * calCurrentPGA_gain * sqrt(2) * ADE9000_RMS_FULL_SCALE_CODES; //(the gain is for all channels the same)
    expectedIRMS = (int32_t) temp; //rounding off  

    //computing the threshold value in decimal code for xIRMS (minimal_iNom threshold value)
    temp = (float)minimal_iNom / (float)iRms_conversionConstant; 
    iNomThresh = (int32_t) temp;

    for (int i=0; i < CUR_CHAN_USED; i++)
    { 
      actualIRMS = ade9000.SPI_Read_32(xIrms_registers_address[i]);          
      if ((actualIRMS > iNomThresh)) //actualIRMS needs to be higher than 0.5Arms in decimal code (the value can be alternated)
      {
        //Calculating the gain and writes to the xIGAIN registers
        temp = (((float)expectedIRMS / (float)actualIRMS) - 1) * 134217728; //calculating the gain
        xIgain_registers[i] = (int32_t) temp; //rounding off
        ade9000.SPI_Write_32(xIgain_register_address[i], xIgain_registers[i]);
        
        //expected xIRMS decimal code value
        Serial.print(" Expected IRMS of current channel ");
        Serial.print(1+i);
        Serial.print("   : ");
        Serial.print(expectedIRMS);
        Serial.println(" decimal code");
        //calculated xIRMS decimal code value
        Serial.print(" Actual IRMS of current channel ");
        Serial.print(1+i);
        Serial.print("    : ");
        Serial.print(actualIRMS);
        Serial.println(" decimal code");
        //xIGAIN decimal code value
        Serial.print(" IGAIN register for current channel "); 
        Serial.print(1+i);
        Serial.print(" : ");
        Serial.print(xIgain_registers[i]);
        Serial.println(" decimal code");
        //deviation floating point value
        Serial.print(" IRMS deviation before calibration: "); 
        Serial.print(1+i);
        Serial.print(" : ");
        Serial.print(((1 - ((float)actualIRMS / (float)expectedIRMS))) * 100);
        Serial.println(" (%)");
        Serial.println(); //spacer to get more distinction between other channels
      }
      else
      {
        Serial.println(" Measured signal below I_nominal threshold value ");
        Serial.print(" Skipped calibration for current channel "); 
        Serial.println(1+i);
        Serial.println();
      }
    }        
    Serial.println(" Calibration process finished");
    Serial.println();
  }  
  else if (serialReadData == 'Q' || serialReadData == 'q')
  {
    Serial.println("Aborting calibration process");
    Serial.println();
    
    /*REMOVE THIS FOR LOOP WHEN THE MCU CAN STORE THE CALIBRATIONS IN HIS FLASH MEMORY*/
    for (int i = 0; i < CUR_CHAN_USED; i++)
    {
      xIgain_registers[i] = 6111184; //6567216; <-- this value is for the EV-ADE9000 bord //this value is for a deviation of 4.66% and is from the final report of a actual calibration.
      //xIgain_registers[i] = 0;
      ade9000.SPI_Write_32(xIgain_register_address[i], xIgain_registers[i]);
      Serial.println(" !! USING STANDARD CALIBRATION VALUES FOR xIGAIN !!");
    }
    
  return;
  }
  else
  {
    Serial.println(" Wrong input");
    ADE9000_current_gain_calibration();
  }
}
