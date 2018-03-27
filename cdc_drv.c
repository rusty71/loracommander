#include "uart.h"
#include "cdc_drv.h"
#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"
#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>

static uint8_t single_desc_bytes[] = {
    CDCD_ACM_DESCES_LS_FS};

#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}};

/* endpoint buffer. choose smart CDCD_ECHO_BUF_SIZ  in Atmel Start */
static uint32_t usbd_cdc_rx_buffer[CDCD_ECHO_BUF_SIZ / 4];
static uint32_t usbd_cdc_tx_buffer[CDCD_ECHO_BUF_SIZ / 4];

/* usb xfer flags */
static volatile bool read_pending = false;
static volatile bool write_pending = false;

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

//serving read() and write()
static QueueHandle_t   rxQ, txQ;

/**
 * \brief Callback invoked when bulk OUT data received
 */
 //TODO:try USB flow control, do not restart read when rxQ < CDCD_ECHO_BUF_SIZ
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    //rx ISR
    BaseType_t queue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(rxQ != NULL) {
        for( int i = 0; i < count; i++) {
            if((queue = xQueueSendFromISR( rxQ, &((uint8_t*)usbd_cdc_rx_buffer)[i], &xHigherPriorityTaskWoken )) != pdTRUE) {
                DEBUG_PRINT("*");
                break;  //TODO: DATA underrun  XOFF?
            }
        }
    }
        cdcdf_acm_read((uint8_t *)usbd_cdc_rx_buffer, sizeof(usbd_cdc_rx_buffer));

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken ); //CHECK:this sets the scheduler tick interrupt flag so scheduler gets called after return from interrupt
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int i = 0;
    
    for( i = 0; i < CDCD_ECHO_BUF_SIZ; i++ ){   //max buf size
        if(xQueueReceiveFromISR( txQ, &(((uint8_t*)usbd_cdc_tx_buffer)[i]), &xHigherPriorityTaskWoken) != pdTRUE) {
        //~ if(xQueueReceiveFromISR( txQ, &(((uint8_t*)usbd_cdc_tx_buffer)[i]), NULL) != pdTRUE) {
            //~ UART_println("q empty");
            break;  //queue empty
        }
        else
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );     //CHECKME
    }
    if(i > 0) {
        write_pending = true;
        cdcdf_acm_write((uint8_t *)usbd_cdc_tx_buffer, i);
    }
    else {
        write_pending = false;
    }

	return false;
}
    
    
static TaskHandle_t xTaskDTRNotify = NULL;

void cdc_wait_DTR(void)
{
    uint32_t ulNotificationValue;
    //~ const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20000 );

    DEBUG_PRINT("WAIT for DTR");

    xTaskDTRNotify = xTaskGetCurrentTaskHandle();

    ulNotificationValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
    DEBUG_PRINT("NOTIFIED");

    if( ulNotificationValue == 1 )
    {
        /* The transmission ended as expected. */
    }
    else
    {
        /* The call to ulTaskNotifyTake() timed out. */
    }
}

static TaskHandle_t xTaskCDCEnabledNotify = NULL;
static cdc_enabled = false;
//blocks until USB host connected to CDC endpoint
void cdc_wait_CDC(void)
{
    uint32_t ulNotificationValue;
    //~ const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20000 );

    if(cdc_enabled)
        return;

    xTaskCDCEnabledNotify = xTaskGetCurrentTaskHandle();
    DEBUG_PRINT("Wait for CDC enable");

    ulNotificationValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

    DEBUG_PRINT("Enabled :");

    if( ulNotificationValue == 1 )
    {
        /* The transmission ended as expected. */
    }
    else
    {
        /* The call to ulTaskNotifyTake() timed out. */
    }
}


/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DEBUG_PRINT("state changed X:");
	if (state.rs232.DTR) {
        DEBUG_PRINT("+DTRX");
        if( xTaskDTRNotify != NULL ) {
            vTaskNotifyGiveFromISR( xTaskDTRNotify, &xHigherPriorityTaskWoken );
            xTaskDTRNotify = NULL;
		}
        /* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)usbd_cdc_rx_buffer, sizeof(usbd_cdc_rx_buffer));
	}
    else {
        DEBUG_PRINT("-DTR");
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	/* No error. */
	return false;
}

static bool usb_device_cb_line_coding_c(const usb_cdc_line_coding_t* coding)
{
    DEBUG_PRINT("linecoding changed: ");
    //~ reset_on_disconnect = coding->dwDTERate == 1200;
    /* Ok to change. */
    return true;
}

static bool usb_device_cb_enable_c(bool enabled)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DEBUG_PRINT("CDC state:");
    if(enabled) {
        cdc_enabled = true;
        if( xTaskCDCEnabledNotify != NULL ) {
            vTaskNotifyGiveFromISR( xTaskCDCEnabledNotify, &xHigherPriorityTaskWoken );
            xTaskCDCEnabledNotify = NULL;
        }
        DEBUG_PRINT("TRUE");
    }
    else {
        cdc_enabled = false;

        DEBUG_PRINT("FALSE");
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	/* No error. */
    return true;
}
/**
 * \brief CDC ACM Init
 */
void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

//TODO: optimize, bypass queue when !write_pending
uint16_t cdc_write(uint8_t *buf, uint16_t len) {
    BaseType_t queue_result;
    int i = 0;

    for( i = 0; i < len; i++ ){ //non blocking
        if((queue_result = xQueueSend( txQ, &(buf[i]), 0 )) != pdTRUE)
            break;  //queue filled
    }
    if(!write_pending) {
        write_pending = true;   //TODO: critical section?
        cdcdf_acm_write((uint8_t *)usbd_cdc_tx_buffer, 0);      //trigger initiate write
    }
    for( ; i < len; i++ ){ //blocking
        if((queue_result = xQueueSend( txQ, &(buf[i]), portMAX_DELAY )) != pdTRUE)
            break;  //queue error
        if(!write_pending) {
            write_pending = true;   //TODO: critical section?
            cdcdf_acm_write((uint8_t *)usbd_cdc_tx_buffer, 0);      //trigger initiate write
        }
    }
    return len;
}


uint16_t cdc_read(uint8_t *buf, uint16_t maxlen, uint32_t timeout) {
    int i;
    uint32_t to = timeout;
    
    for( i = 0; i < maxlen; i++) {
        if( xQueueReceive( rxQ, &(buf[i]), to ) != pdTRUE )
            break;  //queue empty
        to = 0; //dont block on subsequent chars
    }

    return i;
}


void cdc_init(void)
{
    DEBUG_PRINT("Init CDC driver");
    rxQ = xQueueCreate( 128, 1 );
    configASSERT(rxQ);
    txQ = xQueueCreate( 64, 1 );
    configASSERT(txQ);

    cdc_device_acm_init();
    cdcdf_acm_register_callback(CDCDF_ACM_CB_ENABLED_C, (FUNC_PTR)usb_device_cb_enable_c);

	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
    cdcdf_acm_register_callback(CDCDF_ACM_CB_LINE_CODING_C, (FUNC_PTR)usb_device_cb_line_coding_c);
    DEBUG_PRINT("CDC initialized");
}
