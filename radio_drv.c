#include "radio_drv.h"
//~ #include "radio.h"
#include "assert.h"

#include "driver_init.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
//~ #include "atmel_start_pins.h"

#include "FreeRTOS.h"
#include "semphr.h"
//~ #include "task.h"
#include "timers.h"

//~ #include <stdint.h>
//~ #include <stdio.h>
#include <string.h>
//~ #include <math.h>

//Lorawan on 868.10, 868.30  868.50 @125kHz(EU)
//defaults
#define RF_FREQUENCY                                868300000 // Hz

#define RF_MID_BAND_THRESH                          525000000
#define TX_OUTPUT_POWER                             0         // dBm
#define LORA_BANDWIDTH                              6         // [7: 125 kHz,
                                                              //  8: 250 kHz,
                                                              //  9: 500 kHz,
                                                              //  10: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         15         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

static SemaphoreHandle_t RadioSem;  //has radio
static QueueHandle_t rxEventQ, txEventQ;         //global

typedef enum
{
    RXQ_DATA_RECEIVED = 0,
    RXQ_CRC_ERROR,
    RXQ_TIMEOUT,
    RXQ_ERROR,
    RXQ_INTERRUPTED,
    TXQ_DATA_SENT,
    TXQ_TIMEOUT,
} Event_t;

typedef struct
{
    Event_t eType;
    void *pvData;
    uint16_t Len;
    int16_t rssi;
    int8_t snr;
} EventQ_t;


#define DIO0_IRQ_PIN PIN_PA14
#define DIO1_IRQ_PIN PIN_PA20
#define DIO2_IRQ_PIN PIN_PA21 

#define FREQ_STEP 61.03515625
#define RX_BUFFER_SIZE                              256

typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}                                                 \

typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

struct io_descriptor *io;

const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

uint8_t RxTxBuffer[RX_BUFFER_SIZE];

typedef enum
{
    RF_STANDBY = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
}RadioState_t;

typedef struct
{
    uint32_t Channel;
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    uint8_t  Size;       //packet sdize
    int8_t   SnrValue;
    int16_t  RssiValue;
    RadioState_t State;
}RadioLoRaSettings_t;

RadioLoRaSettings_t Settings;

void SPIInit(void) {
	spi_m_sync_enable(&SPI);
	spi_m_sync_get_io_descriptor(&SPI, &io);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size ) {
    uint8_t address = addr & 0x7f;

    gpio_set_pin_level(SPI_RF95_NSS, false); //CS radio
    io_write(io, &address, 1); //write write address
    io_read(io, buffer, size);   //read response
    gpio_set_pin_level(SPI_RF95_NSS, true);
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size ) {
    uint8_t address = addr | 0x80;
    
    //~ UART_println("write1");
    gpio_set_pin_level(SPI_RF95_NSS, false); //CS radio
    io_write(io, &address, 1); //write write address
    io_write(io, buffer, size);   //write data
    gpio_set_pin_level(SPI_RF95_NSS, true);
}


void SX1276Write( uint8_t addr, uint8_t data ) {
    SX1276WriteBuffer(addr, &data, 1);
}

uint8_t SX1276Read( uint8_t addr ) {
    uint8_t data;
    SX1276ReadBuffer( addr, &data, 1 );
    return data;
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size ) {
    SX1276WriteBuffer(0, buffer, size);
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size ) {
    SX1276ReadBuffer(0, buffer, size);
}    



void SX1276Reset( void )
{
    // Set RESET pin to 0
	gpio_set_pin_level(RF95_RESET, false);

    DEBUG_PRINT("RESET");
    //~ vTaskDelay(1000/portTICK_PERIOD_MS);
    volatile TickType_t now;
    now = xTaskGetTickCount();
    while((xTaskGetTickCount()<(now+2)))
        ;
    DEBUG_PRINT("RESET");

    // Configure RESET as input
	gpio_set_pin_level(RF95_RESET, true);

    // Wait 6 ms
    //~ vTaskDelay(600/portTICK_PERIOD_MS);
    now = xTaskGetTickCount();
    while((xTaskGetTickCount()<(now+7)))
        ;
    DEBUG_PRINT("RESET");
}

void SX1276SetChannel( uint32_t freq )
{
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )SX1276Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}

void SX1276SetAntSw( uint8_t rxTx )
{
    if( rxTx != 0 ) // 1: TX, 0: RX
    {
        //~ GpioWrite( &AntSwitchLf, 0 );
        //~ GpioWrite( &AntSwitchHf, 1 );
    }
    else
    {
        //~ GpioWrite( &AntSwitchLf, 1 );
        //~ GpioWrite( &AntSwitchHf, 0 );
    }
}


void SX1276SetAntSwLowPower( bool status )
{
    //~ if( RadioIsActive != status )
    //~ {
        //~ RadioIsActive = status;
    
        //~ if( status == false )
        //~ {
            //~ SX1276AntSwInit( );
        //~ }
        //~ else
        //~ {
            //~ SX1276AntSwDeInit( );
        //~ }
    //~ }
}

void SX1276SetOpMode( uint8_t opMode )
{
    if( opMode == RF_OPMODE_SLEEP )
    {
        SX1276SetAntSwLowPower( true );
    }
    else
    {
        SX1276SetAntSwLowPower( false );
        if( opMode == RF_OPMODE_TRANSMITTER )
        {
            SX1276SetAntSw( 1 );
        }
        else
        {
            SX1276SetAntSw( 0 );
        }
    }
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276OnDio0Irq( void )
{
    volatile EventQ_t rxnotify;

    volatile int8_t irqFlags = 0;
    //~ UART_println("IRQ");
    irqFlags = SX1276Read( REG_LR_IRQFLAGS );
    //~ UART_printInt(irqFlags);

    //clear all interrupts
    SX1276Write( REG_LR_IRQFLAGS, irqFlags );


    if(irqFlags & RFLR_IRQFLAGS_RXDONE) {
        //~ UART_println("RX DONE");
        int8_t snr = 0;
    //~ gpio_set_pin_level(LED_YELLOW,true);

        // Clear Irq
        //~ SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );   //CHECKME:why here/now?

        //~ UART_print('?????????????????????');
        if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
            rxnotify.eType = RXQ_CRC_ERROR;
        }
        else {
            rxnotify.eType = RXQ_DATA_RECEIVED;
        }

        Settings.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
        if( Settings.SnrValue & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            snr = ( ( ~Settings.SnrValue + 1 ) & 0xFF ) >> 2;
            snr = -snr;
        }
        else
        {
            // Divide by 4
            snr = ( Settings.SnrValue & 0xFF ) >> 2;
        }

        int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
        if( snr < 0 )
        {
            if( Settings.Channel > RF_MID_BAND_THRESH )
            {
                Settings.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) + snr;
            }
            else
            {
                Settings.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + snr;
            }
        }
        else
        {
            if( Settings.Channel > RF_MID_BAND_THRESH )
            {
                Settings.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
            }
            else
            {
                Settings.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
            }
        }

        Settings.Size = SX1276Read( REG_LR_RXNBBYTES );
        SX1276ReadFifo( RxTxBuffer, Settings.Size );
        rxnotify.pvData = RxTxBuffer;

        //~ UART_print('?');
        //~ UART_printInt(RxTxBuffer);
        rxnotify.Len = Settings.Size;
        rxnotify.rssi = Settings.RssiValue;
        rxnotify.snr = Settings.SnrValue;
        xQueueSendFromISR( rxEventQ, &rxnotify, 0 );   //CHECKME: overwrite?
    }

    if(irqFlags & RFLR_IRQFLAGS_TXDONE) {
        //~ UART_println("TX done irq");
        EventQ_t txnotify;

        txnotify.eType = TXQ_DATA_SENT;
        txnotify.pvData = NULL;
        txnotify.Len = 0;
        xQueueSendFromISR( txEventQ, &txnotify, 0 );
    }
    //~ ASSERT(false);
    //~ volatile uint8_t irqFlags = 0;

    //~ switch( SX1276.Settings.State )
    //~ {
        //~ case RF_RX_RUNNING:
            //~ DEBUG_PRINT("RXRUNNING");

            //~ switch( SX1276.Settings.Modem )
            //~ {
            //~ case MODEM_FSK:
                //~ if( SX1276.Settings.Fsk.CrcOn == true )
                //~ {
                    //~ irqFlags = SX1276Read( REG_IRQFLAGS2 );
                    //~ if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    //~ {
                        //~ // Clear Irqs
                        //~ SX1276Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    //~ RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    //~ RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        //~ SX1276Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        //~ TimerStop( RxTimeoutTimer );

                        //~ if( SX1276.Settings.Fsk.RxContinuous == false )
                        //~ {
                            //~ TimerStop( RxTimeoutSyncWord );
                            //~ SX1276.Settings.State = RF_IDLE;
                        //~ }
                        //~ else
                        //~ {
                            //~ // Continuous mode restart Rx chain
                            //~ SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                            //~ TimerStart( RxTimeoutSyncWord );
                        //~ }

                        //~ if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        //~ {
                            //~ RadioEvents->RxError( );
                        //~ }
                        //~ SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                        //~ SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                        //~ SX1276.Settings.FskPacketHandler.NbBytes = 0;
                        //~ SX1276.Settings.FskPacketHandler.Size = 0;
                        //~ break;
                    //~ }
                //~ }

                //~ // Read received packet size
                //~ if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                //~ {
                    //~ if( SX1276.Settings.Fsk.FixLen == false )
                    //~ {
                        //~ SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                    //~ }
                    //~ else
                    //~ {
                        //~ SX1276.Settings.FskPacketHandler.Size = SX1276Read( REG_PAYLOADLENGTH );
                    //~ }
                    //~ SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    //~ SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                //~ }
                //~ else
                //~ {
                    //~ SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    //~ SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                //~ }

                //~ if( SX1276.Settings.Fsk.RxContinuous == false )
                //~ {
                    //~ SX1276.Settings.State = RF_IDLE;
                    //~ TimerStart( RxTimeoutSyncWord );
                //~ }
                //~ else
                //~ {
                    //~ // Continuous mode restart Rx chain
                    //~ SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                //~ }
                //~ TimerStop( RxTimeoutTimer );
                //~ if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                //~ {
                    //~ RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.FskPacketHandler.Size, SX1276.Settings.FskPacketHandler.RssiValue, 0 );
                //~ }
                //~ SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                //~ SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                //~ SX1276.Settings.FskPacketHandler.NbBytes = 0;
                //~ SX1276.Settings.FskPacketHandler.Size = 0;
                //~ break;
            //~ case MODEM_LORA:
                //~ {    
                    //~ DEBUG_PRINT("LORA");

                    //~ int8_t snr = 0;

                    // Clear Irq
                    //~ SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    //~ irqFlags = SX1276Read( REG_LR_IRQFLAGS );
                    //~ if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    //~ {
                        //~ // Clear Irq
                        //~ SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        //~ if( Settings.RxContinuous == false )
                        //~ {
                            //~ SX1276.Settings.State = RF_IDLE;
                        //~ }                        
                        //~ TimerStop( RxTimeoutTimer );
                        //~ if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        //~ {
                            //~ RadioEvents->RxError( );
                        //~ }
                        //~ break;
                    //~ }

                    //~ SettingsPacketHandler.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
                    //~ if( SettingsPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    //~ {
                        //~ // Invert and divide by 4
                        //~ snr = ( ( ~SettingsPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        //~ snr = -snr;
                    //~ }
                    //~ else
                    //~ {
                        //~ // Divide by 4
                        //~ snr = ( SettingsPacketHandler.SnrValue & 0xFF ) >> 2;
                    //~ }

                    //~ int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
                    //~ if( snr < 0 )
                    //~ {
                        //~ if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        //~ {
                            //~ SettingsPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          //~ snr;
                        //~ }
                        //~ else
                        //~ {
                            //~ SettingsPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          //~ snr;
                        //~ }
                    //~ }
                    //~ else
                    //~ {
                        //~ if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        //~ {
                            //~ SettingsPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        //~ }
                        //~ else
                        //~ {
                            //~ SettingsPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        //~ }
                    //~ }

                    //~ SettingsPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
                    //~ SX1276ReadFifo( RxTxBuffer, SettingsPacketHandler.Size );

                    //~ if( Settings.RxContinuous == false )
                    //~ {
                        //~ SX1276.Settings.State = RF_IDLE;
                    //~ }
                    //~ TimerStop( RxTimeoutTimer );
                    //~ if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    //~ {
                        //~ RadioEvents->RxDone( RxTxBuffer, SettingsPacketHandler.Size, SettingsPacketHandler.RssiValue, SettingsPacketHandler.SnrValue );
                    //~ }
                //~ }
                //~ break;
            //~ default:
                //~ break;
            //~ }
            //~ break;
        //~ case RF_TX_RUNNING:
            //~ TimerStop( TxTimeoutTimer );

            //~ // TxDone interrupt
            //~ switch( SX1276.Settings.Modem )
            //~ {
            //~ case MODEM_LORA:
                //~ // Clear Irq
                //~ SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                //~ // Intentional fall through
            //~ case MODEM_FSK:
            //~ default:
                //~ SX1276.Settings.State = RF_IDLE;
                //~ if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                //~ {
                    //~ RadioEvents->TxDone( );
                //~ }
                //~ break;
            //~ }
            //~ break;
        //~ default:
            //~ DEBUG_PRINT("UNKNOWN");

            //~ break;
    //~ }
}

void SX1276SetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetOpMode( RF_OPMODE_SLEEP );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetOpMode( RF_OPMODE_SLEEP );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX1276Init( void )
{
    uint8_t i;
    
    SPIInit();
    UART_print("SPI:");
    UART_printInt(SX1276Read(0x42));
    UART_println("..");

    SX1276Reset( );

    RxChainCalibration( );

    //~ SX1276SetOpMode( RF_OPMODE_SLEEP );

	ext_irq_register(DIO0_IRQ_PIN, SX1276OnDio0Irq);
	//~ ext_irq_register(DIO1_IRQ_PIN, SX1276OnDio1Irq);
	//~ ext_irq_register(DIO2_IRQ_PIN, SX1276OnDio2Irq);

    //~ SX1276IoIrqInit( DioIrq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( MODEM_LORA );
}


void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276SetModem( modem );
    //~ if( bandwidth > 2 )
    //~ {
        //~ // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        //~ while( 1 );
    //~ }
    //~ bandwidth += 7;
    Settings.Bandwidth = bandwidth;
    Settings.Datarate = datarate;
    Settings.Coderate = coderate;
    Settings.PreambleLen = preambleLen;
    Settings.FixLen = fixLen;
    Settings.PayloadLen = payloadLen;
    Settings.CrcOn = crcOn;
    Settings.FreqHopOn = freqHopOn;
    Settings.HopPeriod = hopPeriod;
    Settings.IqInverted = iqInverted;
    Settings.RxContinuous = rxContinuous;

    if( datarate > 12 )
    {
        datarate = 12;
    }
    else if( datarate < 6 )
    {
        datarate = 6;
    }

    if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
    {
        Settings.LowDatarateOptimize = 0x01;
    }
    else
    {
        Settings.LowDatarateOptimize = 0x00;
    }

    SX1276Write( REG_LR_MODEMCONFIG1,
                 ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                   RFLR_MODEMCONFIG1_BW_MASK &
                   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                   ( bandwidth << 4 ) | ( coderate << 1 ) |
                   fixLen );

    SX1276Write( REG_LR_MODEMCONFIG2,
                 ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                   RFLR_MODEMCONFIG2_SF_MASK &
                   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                   RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                   ( datarate << 4 ) | ( crcOn << 2 ) |
                   ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

    SX1276Write( REG_LR_MODEMCONFIG3,
                 ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                   ( Settings.LowDatarateOptimize << 3 ) );

    SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

    SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
    SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

    if( fixLen == 1 )
    {
        SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );
    }

    if( Settings.FreqHopOn == true )
    {
        SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
        SX1276Write( REG_LR_HOPPERIOD, Settings.HopPeriod );
    }

    if( ( bandwidth == 9 ) && ( Settings.Channel > RF_MID_BAND_THRESH ) )
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_TEST36, 0x02 );
        SX1276Write( REG_LR_TEST3A, 0x64 );
    }
    else if( bandwidth == 9 )
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_TEST36, 0x02 );
        SX1276Write( REG_LR_TEST3A, 0x7F );
    }
    else
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_TEST36, 0x03 );
    }

    if( datarate == 6 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                       RFLR_DETECTIONOPTIMIZE_MASK ) |
                       RFLR_DETECTIONOPTIMIZE_SF6 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF6 );
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                     RFLR_DETECTIONOPTIMIZE_MASK ) |
                     RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
    DEBUG_PRINT("RX config set");
}

void SX1276SetRx(void)  //lora only
{

    if(Settings.IqInverted == true )
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
    if( Settings.Bandwidth < 9 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
        SX1276Write( REG_LR_TEST30, 0x00 );
        switch( Settings.Bandwidth )
        {
        case 0: // 7.8 kHz
            SX1276Write( REG_LR_TEST2F, 0x48 );
            SX1276SetChannel(Settings.Channel + 7.81e3 );
            break;
        case 1: // 10.4 kHz
            SX1276Write( REG_LR_TEST2F, 0x44 );
            SX1276SetChannel(Settings.Channel + 10.42e3 );
            break;
        case 2: // 15.6 kHz
            SX1276Write( REG_LR_TEST2F, 0x44 );
            SX1276SetChannel(Settings.Channel + 15.62e3 );
            break;
        case 3: // 20.8 kHz
            SX1276Write( REG_LR_TEST2F, 0x44 );
            SX1276SetChannel(Settings.Channel + 20.83e3 );
            break;
        case 4: // 31.2 kHz
            SX1276Write( REG_LR_TEST2F, 0x44 );
            SX1276SetChannel(Settings.Channel + 31.25e3 );
            break;
        case 5: // 41.4 kHz
            SX1276Write( REG_LR_TEST2F, 0x44 );
            SX1276SetChannel(Settings.Channel + 41.67e3 );
            break;
        case 6: // 62.5 kHz
            SX1276Write( REG_LR_TEST2F, 0x40 );
            break;
        case 7: // 125 kHz
            SX1276Write( REG_LR_TEST2F, 0x40 );
            break;
        case 8: // 250 kHz
            SX1276Write( REG_LR_TEST2F, 0x40 );
            break;
        }
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
    }

    SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                      //RFLR_IRQFLAGS_RXDONE |
                                      //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      RFLR_IRQFLAGS_TXDONE |
                                      RFLR_IRQFLAGS_CADDONE |
                                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                      RFLR_IRQFLAGS_CADDETECTED );

    // DIO0=RxDone
    SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

    //~ DEBUG_PRINT("reset fifo");
    SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    //continious
    SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
    
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_PABOOST;
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}


void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    SX1276SetModem( modem );

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );

    DEBUG_PRINT("Set LORA");
    Settings.Power = power;
    //~ if( bandwidth > 2 )
    //~ {
        //~ // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        //~ while( 1 );
    //~ }
    //~ bandwidth += 7;
    Settings.Bandwidth = bandwidth;
    Settings.Datarate = datarate;
    Settings.Coderate = coderate;
    Settings.PreambleLen = preambleLen;
    Settings.FixLen = fixLen;
    Settings.FreqHopOn = freqHopOn;
    Settings.HopPeriod = hopPeriod;
    Settings.CrcOn = crcOn;
    Settings.IqInverted = iqInverted;
    Settings.TxTimeout = timeout;

    if( datarate > 12 )
    {
        datarate = 12;
    }
    else if( datarate < 6 )
    {
        datarate = 6;
    }
    if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
    {
        Settings.LowDatarateOptimize = 0x01;
    }
    else
    {
        Settings.LowDatarateOptimize = 0x00;
    }

    if( Settings.FreqHopOn == true )
    {
        SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
        SX1276Write( REG_LR_HOPPERIOD, Settings.HopPeriod );
    }

    SX1276Write( REG_LR_MODEMCONFIG1,
                 ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                   RFLR_MODEMCONFIG1_BW_MASK &
                   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                   ( bandwidth << 4 ) | ( coderate << 1 ) |
                   fixLen );

    SX1276Write( REG_LR_MODEMCONFIG2,
                 ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                   RFLR_MODEMCONFIG2_SF_MASK &
                   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                   ( datarate << 4 ) | ( crcOn << 2 ) );

    SX1276Write( REG_LR_MODEMCONFIG3,
                 ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                   ( Settings.LowDatarateOptimize << 3 ) );

    SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
    SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

    if( datarate == 6 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                       RFLR_DETECTIONOPTIMIZE_MASK ) |
                       RFLR_DETECTIONOPTIMIZE_SF6 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF6 );
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                     RFLR_DETECTIONOPTIMIZE_MASK ) |
                     RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
    DEBUG_PRINT("TX set");

}

void SX1276SetTx( void )
{

    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                      RFLR_IRQFLAGS_RXDONE |
                                      RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      //RFLR_IRQFLAGS_TXDONE |
                                      RFLR_IRQFLAGS_CADDONE |
                                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                      RFLR_IRQFLAGS_CADDETECTED );

    // DIO0=TxDone
    SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );

    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );

    DEBUG_PRINT("TX mode");

}


void SX1276Send( uint8_t *buffer, uint8_t size )
{

    if( Settings.IqInverted == true )
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    Settings.Size = size;

    //~ // Initializes the payload size
    SX1276Write( REG_LR_PAYLOADLENGTH, size );

    //~ // Full buffer used for Tx
    SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
    {
        SX1276SetOpMode( RF_OPMODE_STANDBY );       //wake up
        volatile TickType_t now;
        now = xTaskGetTickCount();
        while((xTaskGetTickCount()<(now+2)))
            ;

        //~ vTaskDelay(1/portTICK_PERIOD_MS);
    }
    // Write payload buffer
    SX1276WriteFifo( buffer, size );

    SX1276SetTx();
}

static TaskHandle_t xTaskStateNotify = NULL;


uint16_t radio_read(uint8_t *data, uint16_t maxlen, int16_t *rssi, int8_t *snr, TickType_t timeout) {
    EventQ_t  xReceivedStructure;
    BaseType_t xStatus;
    uint16_t ret = 0;


    while(Settings.State != RF_STANDBY) {
        DEBUG_PRINT("Not IDLE");
        xTaskStateNotify = xTaskGetCurrentTaskHandle();
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    }

    if( xSemaphoreTake( RadioSem, timeout ) == pdTRUE ) {
        DEBUG_PRINT("read(): got control");
        //~ ASSERT(Settings.State == RF_RX_RUNNING);
        Settings.State = RF_RX_RUNNING;
        xQueueReset(rxEventQ);      //clear event queue
        SX1276SetRx();
        DEBUG_PRINT("read(): RX started");
        xSemaphoreGive( RadioSem );       //give back radio control, TX might interrupt
        if( xQueueReceive( rxEventQ, &xReceivedStructure, timeout ) == pdPASS ) {  //TODO: subtract already elapsed time?
            DEBUG_PRINT("read(): event");
            if( xReceivedStructure.eType == RXQ_DATA_RECEIVED ) {
                DEBUG_PRINT("RECV");
                //copy to caller
                if(xReceivedStructure.Len>maxlen) //TODO:packet length error
                    ret = maxlen;
                else
                    ret = xReceivedStructure.Len;
                *rssi = xReceivedStructure.rssi;
                *snr = xReceivedStructure.snr;
                DEBUG_PRINT(xReceivedStructure.pvData);
                memcpy(data, xReceivedStructure.pvData, ret);
            }
            else if( xReceivedStructure.eType == RXQ_TIMEOUT ) {
                DEBUG_PRINT("timeout");
            }
            else if( xReceivedStructure.eType == RXQ_INTERRUPTED ) {
                DEBUG_PRINT("interrupted");
            }
        }
    }

    return ret;
}

uint16_t radio_write(uint8_t *data, uint16_t len){
    EventQ_t rxnotify;
    EventQ_t xReceivedStructure;
    BaseType_t xStatus;
    uint16_t ret = 0;


    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //RX running, send interrupt event
        if(Settings.State == RF_RX_RUNNING) {
            Settings.State = RF_TX_RUNNING; //Dont allow read() to take control after interrupt event
            rxnotify.eType = RXQ_INTERRUPTED;
            rxnotify.pvData = NULL;
            rxnotify.Len = 0;
            xQueueSend( rxEventQ, &rxnotify, 0 );   //CHECKME: overwrite?
        }
        DEBUG_PRINT("GOT CONTROL");
        //always return to standby first
        SX1276SetOpMode( RF_OPMODE_STANDBY );   //CHECKME: needed?
        SX1276Send( data, len );                //copies data to fifo, fixed timeout
        xStatus = xQueueReceive( txEventQ, &xReceivedStructure, 10000 );    //no timeout
        DEBUG_PRINT("EVENT");
        
        if( xStatus == pdPASS ) {
            if( xReceivedStructure.eType == TXQ_DATA_SENT ) {
                ret = len;
                Settings.State = RF_STANDBY;
                xTaskNotifyGive( xTaskStateNotify );
                DEBUG_PRINT("SENT");
            }
            else if (xReceivedStructure.eType == TXQ_TIMEOUT ) {
                Settings.State = RF_STANDBY;
                xTaskNotifyGive( xTaskStateNotify );
                DEBUG_PRINT("SENT timeout");
            }
        
        }
        else {
            configASSERT(false);
        }
        Settings.State = RF_STANDBY;
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else {
        configASSERT(false);
    }

    return ret;
}


//keep shadow copy of settings 
//~ static RadioLoRaSettings_t radio_settings;

//~ void radio_txconfig(RadioLoRaSettings_t *settings) {
    //~ if((settings->Power    == radio_settings.Power)      &&
    //~ (settings->Bandwidth   == radio_settings.Bandwidth)  &&
    //~ (settings->Datarate    == radio_settings.Datarate)   &&
    //~ (settings->Coderate    == radio_settings.Coderate)   &&
    //~ (settings->PreambleLen == radio_settings.PreambleLen)&&
    //~ (settings->FixLen      == radio_settings.FixLen)     &&
    //~ (settings->CrcOn       == radio_settings.CrcOn)      &&
    //~ (settings->FreqHopOn   == radio_settings.FreqHopOn)  &&
    //~ (settings->TxTimeout   == radio_settings.TxTimeout))
        //~ return;     //no change
    //~ if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //~ //copy RX settings in shadow
        //~ radio_settings.Power       = settings->Power;
        //~ radio_settings.Bandwidth   = settings->Bandwidth;
        //~ radio_settings.Datarate    = settings->Datarate;
        //~ radio_settings.Coderate    = settings->Coderate;
        //~ radio_settings.PreambleLen = settings->PreambleLen;
        //~ radio_settings.FixLen      = settings->FixLen;
        //~ radio_settings.CrcOn       = settings->CrcOn;
        //~ radio_settings.FreqHopOn   = settings->FreqHopOn;
        //~ radio_settings.TxTimeout   = settings->TxTimeout;    //TODO:TimeOnAir

        //~ Radio.SetTxConfig( MODEM_LORA, settings->Power,
                                       //~ 0, 
                                       //~ settings->Bandwidth,
                                       //~ settings->Datarate, 
                                       //~ settings->Coderate,
                                       //~ settings->PreambleLen, 
                                       //~ settings->FixLen,
                                       //~ settings->CrcOn,
                                       //~ settings->FreqHopOn,
                                       //~ 0, 
                                       //~ LORA_IQ_INVERSION_ON, 
                                       //~ settings->TxTimeout );
        //~ xSemaphoreGive( RadioSem );       //give back radio control
    //~ }
    //~ else
        //~ configASSERT(false);
//~ }

//~ void radio_rxconfig(RadioLoRaSettings_t *settings) {

    //~ if((settings->Bandwidth== radio_settings.Bandwidth)  &&
    //~ (settings->Datarate    == radio_settings.Datarate)   &&
    //~ (settings->Coderate    == radio_settings.Coderate)   &&
    //~ (settings->PreambleLen == radio_settings.PreambleLen)&&
    //~ (settings->FixLen      == radio_settings.FixLen)     &&
    //~ (settings->CrcOn       == radio_settings.CrcOn)      &&
    //~ (settings->FreqHopOn   == radio_settings.FreqHopOn))
        //~ return;     //no change

    //~ if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //~ //copy RX settings
        //~ radio_settings.Bandwidth   = settings->Bandwidth;
        //~ radio_settings.Datarate    = settings->Datarate;
        //~ radio_settings.Coderate    = settings->Coderate;
        //~ radio_settings.PreambleLen = settings->PreambleLen;
        //~ radio_settings.FixLen      = settings->FixLen;
        //~ radio_settings.CrcOn       = settings->CrcOn;
        //~ radio_settings.FreqHopOn   = settings->FreqHopOn;

        //~ Radio.SetRxConfig( MODEM_LORA, 
                                        //~ settings->Bandwidth, 
                                        //~ settings->Datarate,
                                        //~ settings->Coderate,
                                        //~ 0, 
                                        //~ settings->PreambleLen,
                                        //~ LORA_SYMBOL_TIMEOUT,
                                        //~ settings->FixLen,
                                        //~ 0,      //payload len
                                        //~ settings->CrcOn, 
                                        //~ settings->FreqHopOn,
                                        //~ 0,      //HopPeriod
                                        //~ LORA_IQ_INVERSION_ON, 
                                        //~ true    //rxContinuous
                                        //~ );
        //~ xSemaphoreGive( RadioSem );       //give back radio control
    //~ }
    //~ else
        //~ configASSERT(false);
                                    
//~ }

void radio_status(char * buffer) {
    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //~ Radio.GetStatus(buffer);
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else {
        configASSERT(false);
    }

}

void radio_init(void)
{
    DEBUG_PRINT("radio driver init");

    rxEventQ = xQueueCreate( 1, sizeof(EventQ_t) );
    txEventQ = xQueueCreate( 1, sizeof(EventQ_t) );

    RadioSem = xSemaphoreCreateBinary();    //take radio hardware

    configASSERT(RadioSem);

    //~ bool     LowDatarateOptimize;
    //~ uint8_t  PayloadLen;
    //~ uint8_t  HopPeriod;
    //~ bool     RxContinuous;
    Settings.Channel      = RF_FREQUENCY;
    Settings.Power        = TX_OUTPUT_POWER;
    Settings.Bandwidth    = LORA_BANDWIDTH;
    Settings.Datarate     = LORA_SPREADING_FACTOR;
    Settings.Coderate     = LORA_CODINGRATE;
    Settings.PreambleLen  = LORA_PREAMBLE_LENGTH;
    Settings.FixLen       = LORA_FIX_LENGTH_PAYLOAD_ON;
    Settings.CrcOn        = true;
    Settings.FreqHopOn    = false;
    Settings.TxTimeout    = 3000;
    Settings.IqInverted   = true;
    Settings.Size         = RX_BUFFER_SIZE-1;

    SX1276Init();
    SX1276SetModem(MODEM_LORA);
    SX1276SetChannel( RF_FREQUENCY );
    SX1276SetOpMode( RF_OPMODE_STANDBY );
    Settings.State = RF_STANDBY;

    SX1276SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    SX1276SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    xSemaphoreGive(RadioSem);

    DEBUG_PRINT("radio driver initialized");
}

