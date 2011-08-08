#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>

#include <stdint.h>

#define BAUD 19200
#include <util/setbaud.h>

#include "uart.h"

#include "i2cmaster.h" 

#define DO(cmd) if(cmd) { uart_puts(#cmd); uart_puts(" failed!\n"); return -1; }

#define HI_BYTE(x) (uint8_t)(x>>8)
#define LO_BYTE(x) (uint8_t)(x & 0xFF)

int main(void)
{
    //const float freq = 95.9f;
    const float freq = 88.3f;
    const uint16_t freq_c = (uint16_t)((freq + 10.7f) * 20.f);
    uint8_t clk = 0, curr_clk = 0, rds_byte = 0, rds_byte_cnt = 0;
    
    DDRC = 0;
    PORTC = 0;
    DDRC |= (1 << PC7);
    PORTC |= (1 << PC7);
    
    /* Init */
    uart_init(UBRR_VALUE);
    
    sei();
    
    i2c_init();
    
    uart_buf_puts("starting..\n");

    i2c_start_wait(0xc0+I2C_WRITE);
    
    uart_buf_puts("started\n");
    
    DO(i2c_write(0x1f));
    DO(i2c_write(0xe2));
    DO(i2c_write(0x86));
    DO(i2c_write(0x44));
    DO(i2c_write(0xa4));
    
    i2c_stop();
    
    _delay_ms(100);
    
    uart_buf_puts("done phase 1\n");
    
    i2c_start_wait(0x86+I2C_WRITE);
    
    //DO(i2c_write(0x86));
    DO(i2c_write(0x00));
    DO(i2c_write(0x4e));
    DO(i2c_write(0xd0));
    DO(i2c_write(0x77));
    
    i2c_stop();
    
    _delay_ms(100);
    
    uart_buf_puts("done phase 2\n");
    
    i2c_start_wait(0xc0+I2C_WRITE);
    
    DO(i2c_write(0x08));
    DO(i2c_write(0xf0));
    DO(i2c_write(0x88));
    DO(i2c_write(0x59));
    DO(i2c_write(0xb0));
    
    i2c_stop();
    
    _delay_ms(100);
    
    uart_buf_puts("done phase 3\n");
    
    i2c_start_wait(0xc0+I2C_WRITE);
    
    uart_buf_puti16(freq_c);
    uart_buf_putc('\n');
    uart_buf_puti8(HI_BYTE(freq_c));
    uart_buf_putc('\n');
    uart_buf_puti8(LO_BYTE(freq_c));
    uart_buf_putc('\n');
    
    DO(i2c_write(HI_BYTE(freq_c)));
    DO(i2c_write(LO_BYTE(freq_c)));
    DO(i2c_write(0x88));
    DO(i2c_write(0x59));
    DO(i2c_write(0xb0));
    
    i2c_stop();
    
    _delay_ms(100);
    
    uart_buf_puts("done phase 4\n");
    
    PORTC &= ~(1 << PC7);
    
    /*
     * PC4 RDCL
     * PC3 RDDA
     * PC2 QUAL
     * PC1 ARI
     */
    while(1)
    {
        //_delay_ms(100);
        
//         if(!(PINC & (1<<PC2)))
//         {
//             uart_buf_puts("no quali\n");
//             while(!(PINC & (1<<PC2)));
//             uart_buf_puts("quali!\n");
//         }
//         
// #define ARI_PIN (PC1)
// 
//         if(!(PINC & (1<<ARI_PIN)))
//         {
//             PORTC |= (1<<ARI_PIN); //pullup on
//             //_delay_ms(10);
//             if((PINC & (1<<ARI_PIN)))
//             {
//                 //hochohmig
//                 //uart_buf_puts("no rds\n");
//             }
//             else
//             {
//                 //low
//                 //uart_buf_puts("rds!\n");
//             }
//             PORTC &= ~(1<<ARI_PIN); //pullup off
//         }
//         else
//         {
//             //high
//             //uart_buf_puts("rds & ari!\n");
//         }
        
        curr_clk = (PINC & (1<<PC4)) != 0;
        
        if(curr_clk && !clk)
        {
            rds_byte *= 2;
            if((PINC & (1<<PC3)))
            {
                ++rds_byte;
            }
            
            if(++rds_byte_cnt > 7)
            {
                uart_buf_putc(rds_byte);
                rds_byte = 0;
                rds_byte_cnt = 0;
            }
        }
        
        clk = curr_clk;
    }
    
    /* wird nie erreicht */
    return 0;
}
