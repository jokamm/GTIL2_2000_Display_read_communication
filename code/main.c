#include "gtil2_2000_display_read_com.h"

/********************** IO functions **************************/

void IO_Init(void)
{
    uint z;
    
    // general init
    stdio_init_all();
    
    // init pins
    // test pins
    gpio_init(PIN_LED);     // GP25
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_init(PIN_TEST10);  // GP10
    gpio_set_dir(PIN_TEST10, GPIO_OUT);
    gpio_init(PIN_TEST11);  // GP11
    gpio_set_dir(PIN_TEST11, GPIO_OUT);
    
    // pins to read display communication
    gpio_init(PIN_RX2);     // GP9
    gpio_set_dir(PIN_RX2, GPIO_IN);
    
    // pins to read device_id setting
    gpio_init(PIN_DEV_ID0);     // GP20
    gpio_set_dir(PIN_DEV_ID0, GPIO_IN);
    gpio_init(PIN_DEV_ID1);     // GP19
    gpio_set_dir(PIN_DEV_ID1, GPIO_IN);
    gpio_init(PIN_DEV_ID2);     // GP18
    gpio_set_dir(PIN_DEV_ID2, GPIO_IN);
    
    // pin for PWM
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM); // GP14

    // define pin status
    gpio_put(PIN_LED, 1);
    
    gpio_put(PIN_TEST10, 1);
    gpio_put(PIN_TEST11, 1);    
    
    // init .._buffer
    strcpy(rx1_buffer, version);
    for (z = 0; z < BUFFER_SIZE; z++) 
    {    
        tx0_buffer[z] = 0;
        rx0_buffer[z] = 0;
        tx1_buffer[z] = 0;
        rx1_buffer[z] = 0;
        rx2_buffer[z] = 0;
        rx2_buffer[z + BUFFER_SIZE] = 0;
        
        send_buffer1[z] = 0;
    }
    tx0_in_p = 0;
    tx0_out_p = 0;
    rx0_in_p = 0;
    rx0_out_p = 0;
    rx0_timeout = 0;
    tx1_timeout = 0;
    tx1_in_p = 0;
    tx1_out_p = 0;
    rx1_in_p = 0;
    rx1_out_p = 0;
    rx2_in_p = 0;
    rx2_out_p = 0;
    rx2_timeout = 0;

    // setup ISR
    gpio_set_irq_enabled_with_callback(PIN_RX2, GPIO_IRQ_EDGE_FALL , true, &rx2_ISR);
    
    // setup PWM
    pwm_ctrl_val = 1;           // 1 --> 2us
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM);  // Set the GP14 to be PWM
    pwm_slice   = pwm_gpio_to_slice_num(PIN_PWM_OUT);
    pwm_channel = pwm_gpio_to_channel(PIN_PWM_OUT);
    pwm_set_clkdiv(pwm_slice, 256.0f);  // Setting the divider to slow down the clock (-->500KhZ)
    pwm_set_wrap(pwm_slice, 4095);      // setting the Wrap time to 4095 (8.36 ms)
    pwm_set_enabled(pwm_slice, true);
    pwm_set_chan_level(pwm_slice, pwm_channel, pwm_ctrl_val);  // Setting the duty period (2 us per step)

    // get/initialize x1_device_id
    read_device_id_setting();
    
    // Enable the watchdog, requiring the watchdog to be updated every 200ms or the chip will reboot
    // second arg is when set to pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(200, 0);
}


void check_IO_loop(void)
{
    uint64_t t;
    uint16_t i;
    uint16_t my_crc;
        
//    gpio_put(PIN_TEST11, 0);
    
    
    t = time_us_64();
 
    if ((rx2_rx_in_progress) && (t >= rx2_bit_times[rx2_bit_cnt]))
    {
        rx2_timeout = t + 2000;     // 2 ms
        parse_Rx2();
    }
    else if (t >= tx0_reqest_time)
    {
        // set next tx0_intervall
        tx0_reqest_time = t + tx0_intervall;    // next in 10000000l us - all 10 sec.
       
        // request dataset from WR via UART0
        // set to next tx0_request_idx
        tx0_request_idx++;
        if (tx0_request_idx > 1) tx0_request_idx = 0;
        // init
        tx0_in_p = 0;
        tx0_out_p = 0;
        rx0_in_p = 0;
        // create complete request string
	for (i = 0; i < 6; i++)
        {
            tx0_buffer[tx0_in_p++] = tx0_request_str[tx0_request_idx][i];
        }
		
	my_crc = crc_x0(tx0_buffer, 6);
	
	tx0_buffer[tx0_in_p++] = (uint8_t) (0x00ff & my_crc);
	tx0_buffer[tx0_in_p++] = (uint8_t) (my_crc >> 8);
	tx0_buffer[tx0_in_p++] = 0x0d;
	tx0_buffer[tx0_in_p++] = 0x0a;
        
        // start sending
        UART0_Tx();
        
        // set rx0_timeout
        rx0_timeout = t + 40000;    // check response after 40 ms
        
        // update x1_device_id
        read_device_id_setting();
    }
    else
    {
        watchdog_update();
        
        UART0_Tx();
        UART0_Rx();
        
        UART1_Tx();
        UART1_Rx();
 
        if ((rx0_timeout) && (t > rx0_timeout))
        {
            rx0_timeout = 0;
            
            // validate response
            parse_Rx0();
        }
       
        if (rx1_in_p != rx1_out_p)
        {
            if (rx1_in_p > 6)
            {
                // received a 7 byte for a request
                parse_Rx1();
            }
        }

        if ((tx1_timeout) && (t > tx1_timeout))
        {
            tx1_timeout = 0;
            
            // set X1/RS485 to receice
            gpio_put(UART1_TX_RX_CTRL_PIN, 0);   // set to 0 for RX
        }

        if ((rx1_timeout) && (t > rx1_timeout))
        {
            rx1_timeout += 2000000; // add 2 sec.
            
            if (pwm_ctrl_val > 3) 
            {
                pwm_ctrl_val /= 2;
            }
            else
            {
                pwm_ctrl_val = 1;
                rx1_timeout = 0;
            }
            
            // set PWM value
            if (pwm_ctrl_val > 4000) pwm_ctrl_val = 4000;
            pwm_set_chan_level(pwm_slice, pwm_channel, pwm_ctrl_val);  // Setting the duty period (2 us per step)
        }    
    
        if ((rx2_timeout) && (t > rx2_timeout))
        {
            rx2_timeout = 0;
            
            // copy relevant bytes
            if ((rx2_buffer[0] == 0x07) && (rx2_buffer[1] == 0x07) && 
                ((rx2_buffer[279] == 0x93) || (rx2_buffer[279] == 0x94) || (rx2_buffer[279] == 0x95)))
            {
                // AC Line voltage
                send_buffer1[4] = rx2_buffer[280];
                send_buffer1[5] = rx2_buffer[281];
                
                // Power WR out
                send_buffer1[6] = rx2_buffer[282];
                send_buffer1[7] = rx2_buffer[283];
                
                // DC Batt. voltage
                send_buffer1[8] = rx2_buffer[284];
                send_buffer1[9] = rx2_buffer[285];
                
                // Temp. WR
                send_buffer1[10] = rx2_buffer[292];
                send_buffer1[11] = rx2_buffer[293];
                    
            }
        }
    }
}


void parse_Rx0(void)
{
    uint16_t rx_crc;
    
    if (!tx0_request_idx)
    {   
        // requested energy total
        if ((rx0_buffer[0] == 0x01) && (rx0_buffer[1] == 0x03) && (rx0_buffer[2] == 0x02))
        {
            rx_crc = crc_x0(rx0_buffer, 5);
            if ((rx0_buffer[5] == (uint8_t) (0x00ff & rx_crc)) && (rx0_buffer[6] == (uint8_t) (rx_crc >> 8)))
            {
                // valid data received
                // copy data, Energie Tag
                send_buffer1[12] = rx0_buffer[3];
                send_buffer1[13] = rx0_buffer[4];
            }
        }
    }
    else
    {
        // requested energy day
        if ((rx0_buffer[0] == 0x01) && (rx0_buffer[1] == 0x03) && (rx0_buffer[2] == 0x04))
        {
            rx_crc = crc_x0(rx0_buffer, 7);
            if ((rx0_buffer[7] == (uint8_t) (0x00ff & rx_crc)) && (rx0_buffer[8] == (uint8_t) (rx_crc >> 8)))
            {
                // valid data received
                // copy data, Energie Total
                send_buffer1[16] = rx0_buffer[3];
                send_buffer1[17] = rx0_buffer[4];
                send_buffer1[14] = rx0_buffer[5];
                send_buffer1[15] = rx0_buffer[6];
            }
        }
    }
    
    // init
    tx0_in_p = 0;
    tx0_out_p = 0;
    rx0_in_p = 0;
}


void parse_Rx1(void)
{
    bool found_request_data1 = 0;
    uint8_t a, crc;
    uint8_t rx1_request_str[4];

    // create rx1_request_str 
    rx1_request_str[0] = x1_start_byte;
    rx1_request_str[1] = x1_device_id;
    rx1_request_str[2] = x1_request_data_byte;
    rx1_request_str[3] = x1_request_length_byte;
    
    // there are 7 char in rx1_buffer - compare first 4
    do 
    {
        if (!memcmp(&rx1_buffer[rx1_out_p], rx1_request_str, 4)) 
        {
            // request found
            found_request_data1 = 1;
        }
        rx1_out_p++;
        if (rx1_out_p > BUFFER_SIZE) rx1_out_p = 0; 
    } while ((!found_request_data1) && (rx1_in_p > (rx1_out_p + 6)));

    if (found_request_data1)
    {
        // calculate checksum in rx1_buffer
        crc = 0;
        for (a = 0; a < 6; a++)
        {
            crc += rx1_buffer[a];
        }
        if (rx1_buffer[6] != crc) found_request_data1 = 0;
    }

    if (found_request_data1)
    {
        // get pwm value
        pwm_ctrl_val = rx1_buffer[4] * 256 + rx1_buffer[5];        
        
        // set PWM value
        if (pwm_ctrl_val > 4000) pwm_ctrl_val = 4000;
        pwm_set_chan_level(pwm_slice, pwm_channel, pwm_ctrl_val);  // Setting the duty period (2 us per step)
        
        // valid request received
        transfer_data_for_Tx1();
    }
    
    rx1_in_p = 0;
    rx1_out_p = 0;
}


void parse_Rx2(void)
{
    bool rx2_pin;

    // read pin
    rx2_pin = gpio_get(PIN_RX2);
    
    // rx_bit_cnt 1..8
    switch (rx2_bit_cnt)
    {
        case 0: {   // start bit
                    rx2_in_byte = 0;        // init rx2_in_byte
                    // pin should be 0; if 1 then abort
                    if (rx2_pin == 1) rx2_rx_in_progress = 0;
                }
                break;
            
        case 1: if (rx2_pin) rx2_in_byte |= 0x01;
                break;
            
        case 2: if (rx2_pin) rx2_in_byte |= 0x02;
                break;
            
        case 3: if (rx2_pin) rx2_in_byte |= 0x04;
                break;
            
        case 4: if (rx2_pin) rx2_in_byte |= 0x08;
                break;
            
        case 5: if (rx2_pin) rx2_in_byte |= 0x10;
                break;
            
        case 6: if (rx2_pin) rx2_in_byte |= 0x20;
                break;
            
        case 7: if (rx2_pin) rx2_in_byte |= 0x40;
                break;
            
        case 8: if (rx2_pin) rx2_in_byte |= 0x80;
                break;
            
        case 9: {   // stop bit
                    // should be 1; if 0 then abort
                    if (rx2_pin == 1) 
                    {
                        // byte is complete
                        // check for start of display data string
                        if ((rx2_last_in_byte == 0x07) && (rx2_in_byte == 0x07))
                        {
                            // start received
                            rx2_buffer[0] = 0x07;
                            rx2_buffer[1] = 0x07;
                            rx2_in_p = 2;
                        }
                        else
                        {
                            // copy byte to buffer
                            rx2_buffer[rx2_in_p++] = rx2_in_byte;
                            if (rx2_in_p > (2 * BUFFER_SIZE)) rx2_in_p = 0;
                        }
                        rx2_last_in_byte = rx2_in_byte;
                    }
                    
                    // prepare for next byte
                    rx2_rx_in_progress = 0;
                    rx2_bit_cnt = 0;
                    rx2_in_byte = 0;
                }
                break;
            
        default: {  // should never be here, but just in case
                    // prepare for next char
                    rx2_rx_in_progress = 0;
                    rx2_bit_cnt = 0;
                    rx2_in_byte = 0;
                }
                break;
    }
    
    // set for next bit
    if (rx2_rx_in_progress) rx2_bit_cnt++;
}


/********************** end IO functions **************************/


/********************** helper function **************************/

uint16_t crc_x0(uint8_t *buf, int len)
{
    uint16_t i, pos, crc = 0xffff;
    
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t) buf[pos];
        
        for (i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xa001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return (crc);
}

void transfer_data_for_Tx1(void)
{
    uint z, a = 0;
    uint8_t d0;
    uint64_t t;
    
    // reset
    tx1_in_p = 0;
    tx1_out_p = 0;

    // fixed send_string send_buffer1
    send_buffer1[0] = x1_start_byte;            // fixed start byte
    send_buffer1[1] = x1_device_id;             // request/response device id - range 0x00 .. 0x07
    send_buffer1[2] = x1_response_data_byte;    // requested data id = 0xa0 - response 0xa1
    send_buffer1[3] = x1_response_length_byte;  // data length (fixed)

    // AC Line voltage, Power WR out, DC Batt. voltage, Temp. WR - data comes from X2
    // and is copied into send_buffer1 in check_IO_loop()

    // Energie Tag, Energie gesamt - data comes from X0
    // and is copied into send_buffer1 in parse_Rx0()
    
    // calculate checksum send_buffer1
    d0 = 0;
    for (a = 0; a < (x1_response_length_byte + 4); a++)
    {
        d0 += send_buffer1[a];
    }
    send_buffer1[x1_response_length_byte + 4] = d0;

    // send tx1_buffer
    for (a = 0; a < (x1_response_length_byte + 5); a++)
    {
        tx1_buffer[tx1_in_p++] = send_buffer1[a];
        if (tx1_in_p > BUFFER_SIZE) tx1_in_p = 0;
    }
    
    // set RS485 for send
    gpio_put(UART1_TX_RX_CTRL_PIN, 1);   // set to 1 for TX
    
    // set timeouts
    t = time_us_64();
    tx1_timeout = t + 2000;     // set to + 2ms
    rx1_timeout = t + 2000000;  // set to + 2 sec.
}


void read_device_id_setting(void)
{
    /*  Settings
     * 
     * 1    2    3
     * On   On   On   = 0x00
     * Off  On   On   = 0x01
     * On   Off  On   = 0x02
     * Off  Off  On   = 0x03
     * On   On   Off  = 0x04
     * Off  On   Off  = 0x05
     * On   Off  Off  = 0x06
     * Off  Off  Off  = 0x07
     */
    
    x1_device_id = 0;
    
    if (!gpio_get(PIN_DEV_ID2)) x1_device_id |= 0x01;
    if (!gpio_get(PIN_DEV_ID1)) x1_device_id |= 0x02;
    if (!gpio_get(PIN_DEV_ID0)) x1_device_id |= 0x04;
}
    
/********************** end helper functions **************************/


/********************** UART functions **************************/

void UART0_Init(void)
{
    uart_init(UART0_ID, BAUD_RATE);
    uart_set_format(UART0_ID, 8, 1, UART_PARITY_NONE);
    uart_set_translate_crlf(UART0_ID, false);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
}

void UART0_Rx(void)
{
    if (uart_is_readable(UART0_ID))   // new char in rx fifo ?
    {
        rx0_buffer[rx0_in_p++] = uart_getc(UART0_ID);
        if (rx0_in_p > BUFFER_SIZE) rx0_in_p = 0;
    }
}
 
void UART0_Tx(void)
{
    if (tx0_in_p != tx0_out_p)
    {
        if (uart_is_writable(UART0_ID))
        {
            uart_putc_raw(UART0_ID, tx0_buffer[tx0_out_p++]);  // update UART transmit data register
            if (tx0_out_p > BUFFER_SIZE) tx0_out_p = 0;
        }
    }
}

void UART1_Init(void)
{
    uart_init(UART1_ID, BAUD_RATE);
    uart_set_format(UART1_ID, 8, 1, UART_PARITY_NONE);
    uart_set_translate_crlf(UART1_ID, false);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    gpio_init(UART1_TX_RX_CTRL_PIN);  // GP7
    gpio_set_dir(UART1_TX_RX_CTRL_PIN, GPIO_OUT);
}

void UART1_Rx(void)
{
    if (uart_is_readable(UART1_ID))   // new char in rx fifo ?
    {
        rx1_buffer[rx1_in_p++] = uart_getc(UART1_ID);
        if (rx1_in_p > BUFFER_SIZE) rx1_in_p = 0;
    }
}
 
void UART1_Tx(void)
{
    if (tx1_in_p != tx1_out_p)
    {
        if (uart_is_writable(UART1_ID))
        {
            uart_putc_raw(UART1_ID, tx1_buffer[tx1_out_p++]);  // update UART transmit data register
            if (tx1_out_p > BUFFER_SIZE) tx1_out_p = 0;
            
            tx1_timeout += 1100;
        }
    }
}

void rx2_ISR()
{
    uint64_t t;
    
    if (!rx2_rx_in_progress)
    {
        rx2_rx_in_progress = 1;
        // new char starts, calculate time (in us) for 9600 baud
        t = time_us_64();
        rx2_bit_times[0] = t + 52;
        rx2_bit_times[1] = t + 156;
        rx2_bit_times[2] = t + 260;
        rx2_bit_times[3] = t + 365;
        rx2_bit_times[4] = t + 469;
        rx2_bit_times[5] = t + 573;
        rx2_bit_times[6] = t + 677;
        rx2_bit_times[7] = t + 781;
        rx2_bit_times[8] = t + 885;
        rx2_bit_times[9] = t + 989;
    }   /*
    else
    {
        // do nothing
    }   */
}

/********************** end UART functions **************************/


int main() 
{
    IO_Init();
    UART0_Init();
    UART1_Init();
    
    while (1) 
    {
        /* used for testing 
        gpio_put(PIN_TEST10, 1);
        gpio_put(PIN_TEST11, 1);
        gpio_put(PIN_TEST10, 0);
        gpio_put(PIN_TEST11, 0);
        */

        check_IO_loop();
    }
}
