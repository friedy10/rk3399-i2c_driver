/*
 * Copyright (c) 2025, Friedrich Doku <friedy@u.northwestern.edu>
 *
 * RK3399 I2C Driver implementation for ROCKPRO64 
 * 
 * note: users I2C4 on rockpro64
 */

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <arch.h>
#include <stdint.h>
#include <arch_helpers.h>
#include <lib/pmf/pmf.h>
#include <lib/runtime_instr.h>
#include <plat/common/platform.h>
#include <drivers/gpio.h>
#include <drivers/delay_timer.h>
#include <stdbool.h>
#include <stdio.h>
#include "rk3399_i2c.h"

/*
 * SCL Divisor = 8 * (CLKDIVL+1 + CLKDIVH+1)
 * SCL = PCLK / SCLK Divisor
 * i2c_rate = PCLK
 */

#define debug INFO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshift-overflow"

//void udelay( int usec )
//{
//    int i;
//    unsigned long long tmp;
//    unsigned long long tmp2;
//    unsigned long long timebase_h;
//    unsigned long long timebase_l;
//
//    timebase_l = *TIMER5_CURR_VALUE0;
//    timebase_h = *TIMER5_CURR_VALUE1;
//    tmp = (timebase_h << 32) | timebase_l;
//    tmp2 = 0;
//    for ( i = 0; i < 24; i++ ) {            // timer frequency is 24 MHz
//        tmp2 += usec;
//    }
//    tmp2 += tmp;
//    tmp2 += 1;
//    while( tmp < tmp2 ) {
//        timebase_l = *TIMER5_CURR_VALUE0;
//        timebase_h = *TIMER5_CURR_VALUE1;
//        tmp = (timebase_h << 32) | timebase_l;
//    }
//}

void timer_initx( void )
{
    *TIMER5_LOAD_COUNT2 = 0x0;
    *TIMER5_LOAD_COUNT3 = 0x0;
    *TIMER5_LOAD_COUNT0 = 0xffffffff;
    *TIMER5_LOAD_COUNT1 = 0xffffffff;
    *TIMER5_CTRL_REG    = 0x1;              // free running
}


void rk3399_i2c_set_clk( int scl_rate )
{
    unsigned int src_clk_div;

    src_clk_div = 39;           // GPLL_HZ 800 * MHz / 40 = 20 MHz
    *PMUCRU_CLKSEL_CON3 = (0xffff << 16) | (src_clk_div); 
    *I2C4_CLKDIV = 0x000b000b;  // 100 KHz
}

int rk_i2c_send_start_bit( void )
{
	int TimeOut = I2C_TIMEOUT_US;

        *I2C4_IPD = I2C_IPD_ALL_CLEAN;
        *I2C4_CON = I2C_CON_EN | I2C_CON_START;
        *I2C4_IEN = I2C_STARTIEN;

	while (TimeOut--) {
	    if (*I2C4_IPD & I2C_STARTIPD) {
                *I2C4_IPD = I2C_STARTIPD;
		break;
	    }
	    udelay( 1 );
	}
	if (TimeOut <= 0) {
		return I2C_ERROR_TIMEOUT;
	}

	return I2C_OK;
}

int rk_i2c_send_stop_bit( void )
{
	int TimeOut = I2C_TIMEOUT_US;

        *I2C4_IPD = I2C_IPD_ALL_CLEAN;
        *I2C4_CON = I2C_CON_EN | I2C_CON_STOP;
        *I2C4_IEN = I2C_CON_STOP;

        while (TimeOut--) {
	    if (*I2C4_IPD & I2C_STOPIPD) {
                *I2C4_IPD = I2C_STOPIPD;
	        break;
	    }
	    udelay( 1 );
	}
	if (TimeOut <= 0) {
		return I2C_ERROR_TIMEOUT;
	}

	return I2C_OK;
}

void rk_i2c_disable( void )
{
        *I2C4_CON = 0x0;
}

int rk_i2c_write( char chip, int reg, int r_len, char *buf, int b_len )
{
	int err = I2C_OK;
	int TimeOut = I2C_TIMEOUT_US;
	char *pbuf = buf;
	int bytes_remain_len = b_len + r_len + 1;
	int bytes_tranfered_len = 0;
	int words_tranfered_len = 0;
	int txdata;
	int i, j;

	err = rk_i2c_send_start_bit();
	if (err != I2C_OK) {
	    return err;
	}

        while (bytes_remain_len) {
            if (bytes_remain_len > RK_I2C_FIFO_SIZE) {
	        bytes_tranfered_len = 32;
            } else {
	        bytes_tranfered_len = bytes_remain_len;
            }
            words_tranfered_len = (bytes_tranfered_len + 3) / 4;

	    for (i = 0; i < words_tranfered_len; i++) {
	        txdata = 0;
	        for (j = 0; j < 4; j++) {
	            if ((i * 4 + j) == bytes_tranfered_len) {
		        break;
		    }

		    if (i == 0 && j == 0) {
		        txdata |= (chip << 1);
		    } else if (i == 0 && j <= r_len) {
		        txdata |= (reg & (0xff << ((j - 1) * 8))) << 8;
		    } else {
		        txdata |= (*pbuf++)<<(j * 8);
		    }
                    *(volatile unsigned int *)(I2C4_TXDATA_BASE + i) = txdata;
		}
	    }

            *I2C4_CON = I2C_CON_EN | I2C_CON_MOD(I2C_MODE_TX);
            *I2C4_MTXCNT = bytes_tranfered_len;
            *I2C4_IEN = I2C_MBTFIEN | I2C_NAKRCVIEN;

	    TimeOut = I2C_TIMEOUT_US;
	    while (TimeOut--) {
	        if (*I2C4_IPD & I2C_NAKRCVIPD) {
                    *I2C4_IPD = I2C_NAKRCVIPD;
	            err = I2C_ERROR_NOACK;
	        }
	        if (*I2C4_IPD & I2C_MBTFIPD) {
                    *I2C4_IPD = I2C_MBTFIPD;
	            break;
		}
		udelay( 1 );
	    }

	    if (TimeOut <= 0) {
	        err =  I2C_ERROR_TIMEOUT;
	        goto i2c_exit;
	    }

	    bytes_remain_len -= bytes_tranfered_len;
	}

i2c_exit:
	rk_i2c_send_stop_bit();
	rk_i2c_disable();

	return err;
}


int rk_i2c_send_data(char chip, int reg, const char *data, int len)
{
    int ret;

    ret = rk_i2c_write(chip, reg, 1, (char *)data, len);
    if (ret != I2C_OK) {
        debug("Error sending data to device 0x%02X, error code: %d\n", chip, ret);
    }

    return ret;
}


void rk_i2c_detect(void)
{
    int addr;
    int ret;

    debug("Scanning I2C bus for devices...\n");

    for (addr = 0x03; addr <= 0x77; addr++) {
        debug("before\n");
        ret = rk_i2c_write((char)addr, 0x00, 1, NULL, 0);
        if (ret == I2C_OK) {
            debug("I2C device detected at address 0x%02X\n", addr);
        }
        debug("after\n");
        udelay(10);
    }

    debug("I2C bus scan complete.\n");
}


void test_i2c(void)
{
    *GPIO1B = (0xf << 22) | (0x5 << 6);        // select I2C4
    rk3399_i2c_set_clk(100000);

    while(true){
        rk_i2c_detect();
    }

}
