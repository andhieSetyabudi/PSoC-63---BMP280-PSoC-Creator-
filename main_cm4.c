/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "string.h"

#define I2C_BUFFER_SIZE (10u)
#define BMP280_ADDR 0x76
#define BMP280_ID   0x58

// sensor struct
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

typedef struct config_t_f {
    /** Inactive duration (standby time) in normal mode */
    unsigned int t_sb : 3;
    /** Filter settings */
    unsigned int filter : 3;
    /** Unused - don't set */
    unsigned int none : 1;
    /** Enables 3-wire SPI */
    unsigned int spi3w_en : 1;
    /** Used to retrieve the assembled config register's byte value. */
  }config_t;

 typedef struct ctrl_meas {
    /** Temperature oversampling. */
    unsigned int osrs_t : 3;
    /** Pressure oversampling. */
    unsigned int osrs_p : 3;
    /** Device mode */
    unsigned int mode : 2;
    /** Used to retrieve the assembled ctrl_meas register's byte value. */
  }ctrl_meas_t;

// sensor functions & variable
bmp280_calib_data bmp280_data_cal;
cy_en_scb_i2c_status_t  BMP280_read_calibration(bmp280_calib_data *data);
cy_en_scb_i2c_status_t  BMP280_temp_configure(void);
uint8_t BMP280_read_ID(void);
cy_en_scb_i2c_status_t BMP280_readValue(bmp280_calib_data cal, float* temperature, float*pressure, uint32_t timeout);

#define SYS_TICK_INTERVAL_1mS	(32768/1000)-1
uint32 tickCount;

void SysTickCallback(void) { tickCount++; }
uint32_t getTick()  { return tickCount; }

#define MAX_PRECISION	(6)
static const float rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4,
	0.000005,			// 5,
	0.0000005			// 6
};

char* ftoa(float f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;
	if (precision > MAX_PRECISION)		// check precision bounds
		precision = MAX_PRECISION;
	if (f < 0) {		// sign stuff
		f = -f;
		*ptr++ = '-';
	}
	if (precision < 0) {  // negative precision == automatic precision guess
		if (f < 1.0) precision = 4;
		else if (f < 10.0) precision = 3;
		else if (f < 100.0) precision = 2;
		else if (f < 1000.0) precision = 1;
		else precision = 0;
	}
	if (precision)  	// round value according the precision
		f += rounders[precision];
	// integer part...
	intPart = f;
	f -= intPart;
	if (!intPart)
		*ptr++ = '0';
	else
	{
		p = ptr;	// save start pointer
		while (intPart) { // convert (reverse order)
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}
		p1 = p;  // save end pos
		while (p > ptr)	{ // reverse result
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}
		ptr = p1;	// restore end pos
	}
	if (precision) {	// decimal part
		*ptr++ = '.';	// place decimal point
		while (precision--)	 { // convert
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}
	*ptr = 0;	// terminating zero
	return buf;
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();
    
    UART_PutString("Hello\r\n");
    char buf_txt[25]; char strF[10];
    uint8_t btn_state, pin_state;
    
    // setup systick
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_LF, SYS_TICK_INTERVAL_1mS);
	Cy_SysTick_SetCallback(0,SysTickCallback);
    uint32_t lastTick = 0;
    
    // setup I2C
    I2C_Start();

    // init sensor
    uint8_t sensorID = BMP280_read_ID();
    if( sensorID == BMP280_ID )
    {
        UART_PutString("Device Found !\r\n");
        sprintf(buf_txt,(const char*)("Sensor ID = 0x%x \r\n"), sensorID);
        UART_PutString(buf_txt);
        
        if( BMP280_read_calibration(&bmp280_data_cal) == CY_SCB_I2C_SUCCESS )
        {
            UART_PutString("Load param success !\r\n");
            BMP280_temp_configure();
        }
    }
    float tp, press;
    for(;;)
    {
        if( getTick() - lastTick >= 1000)
        {
            btn_state = Cy_GPIO_Read(button_PORT,button_NUM);
            pin_state = Cy_GPIO_Read(TP_PORT, TP_NUM);
            sprintf(buf_txt,(const char*)("state : %d \t %d \t %d\r\n"), btn_state, pin_state,getTick());
            UART_PutString(buf_txt);
            Cy_GPIO_Inv(P7_1_PORT, P7_1_NUM);
            BMP280_readValue(bmp280_data_cal, &tp, &press, 500);
            sprintf(buf_txt,(const char*)("temperature : %s \t"), ftoa(tp,strF,2));
            UART_PutString(buf_txt);
            sprintf(buf_txt,(const char*)("pressure : %sPa\r\n"), ftoa(press,strF,2));
            UART_PutString(buf_txt);
            lastTick = getTick();
        }
        /* Place your application code here. */
    }
}

cy_en_scb_i2c_status_t bmp_write_bytes(uint8_t *data, uint8_t dataLen, uint32_t timeout)
{
    cy_en_scb_i2c_status_t status;
    I2C_MasterSendStart(BMP280_ADDR,CY_SCB_I2C_WRITE_XFER,timeout);
    for(uint8_t i = 0; i<dataLen; i++)
        status |= I2C_MasterWriteByte(data[i], timeout);
    status = I2C_MasterSendStop(100);
    return status;
}

cy_en_scb_i2c_status_t bmp_write_byte(uint8_t reg, uint8_t data, uint32_t timeout)
{
    cy_en_scb_i2c_status_t status;
    I2C_MasterSendStart(BMP280_ADDR,CY_SCB_I2C_WRITE_XFER,timeout);
    if( CY_SCB_I2C_SUCCESS == I2C_MasterWriteByte(reg, timeout))
    {
        status = I2C_MasterWriteByte(data, timeout);
    }
    status = I2C_MasterSendStop(100);
    return status;
}

cy_en_scb_i2c_status_t bmp_read_bytes(uint8_t reg, uint8_t *val, uint8_t dataLen, uint32_t timeout)
{
    cy_en_scb_i2c_status_t status;
    uint8_t buff=0;
    I2C_MasterSendStart(BMP280_ADDR,CY_SCB_I2C_WRITE_XFER,timeout);
    if( CY_SCB_I2C_SUCCESS == I2C_MasterWriteByte(reg, timeout) )
    {
        I2C_MasterSendReStart(BMP280_ADDR,CY_SCB_I2C_READ_XFER,timeout);
        for( uint8_t i = 0; i< dataLen-1; i++)
             I2C_MasterReadByte(CY_SCB_I2C_ACK, val++, timeout);
        I2C_MasterReadByte(CY_SCB_I2C_NAK, val, timeout);
    }
    status = I2C_MasterSendStop(timeout);
    return CY_SCB_I2C_SUCCESS;
}

cy_en_scb_i2c_status_t bmp_read_byte(uint8_t reg, uint8_t *val, uint32_t timeout)
{
    cy_en_scb_i2c_status_t status;
    uint8_t buff=0;
    I2C_MasterSendStart(BMP280_ADDR,CY_SCB_I2C_WRITE_XFER,timeout);
    if( CY_SCB_I2C_SUCCESS == I2C_MasterWriteByte(reg, timeout) )
    {
        I2C_MasterSendReStart(BMP280_ADDR,CY_SCB_I2C_READ_XFER,timeout);
        status = I2C_MasterReadByte(CY_SCB_I2C_NAK, val, timeout);
    }
    status |= I2C_MasterSendStop(timeout);
    return status;
}

unsigned int config_get(config_t cfg){ 
    return (cfg.t_sb << 5) | (cfg.filter << 2) | cfg.spi3w_en; 
}
unsigned int meas_get(ctrl_meas_t meas) { 
    return (meas.osrs_t << 5) | (meas.osrs_p << 2) | meas.mode; 
}

cy_en_scb_i2c_status_t  BMP280_temp_configure(void)
{
    cy_en_scb_i2c_status_t status;
    config_t t_cfg ={
            .t_sb   = 0x04,
            .filter = 0x04,
    };
    ctrl_meas_t t_meas = {
            .mode   = 0x03,
            .osrs_t = 0x02,
            .osrs_p = 0x05,
    };
    uint8_t buf[4];
    buf[0] = 0xF4;
    buf[1] = meas_get(t_meas);
    buf[2] = 0xF5;
    buf[3] = config_get(t_cfg);
    status = bmp_write_bytes(buf,4,1000);
    return status;
}

uint8_t BMP280_read_ID(void)
{
    uint8_t ret = 0xff;
    if( CY_SCB_I2C_SUCCESS == bmp_read_byte(0xD0, &ret, 1000) )
        return ret;
    else
        return 0xff;
}

cy_en_scb_i2c_status_t BMP280_read_calibration(bmp280_calib_data *data)
{
    cy_en_scb_i2c_status_t status;
    uint8_t buffer_i2c[2];
    status = bmp_read_bytes(0x88, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_T1 = (uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]);
    else
        return status;   
    status = bmp_read_bytes(0x8A, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_T2 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
        
    status = bmp_read_bytes(0x8C, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_T3 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;    

    status = bmp_read_bytes(0x8E, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P1 = (uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]);
    else
        return status;  
        
    status = bmp_read_bytes(0x90, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P2 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;   
    
    status = bmp_read_bytes(0x92, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P3 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x94, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P4 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x96, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P5 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x98, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P6 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x9A, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P7 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x9C, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P8 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;
    
    status = bmp_read_bytes(0x9E, buffer_i2c, 2, 1000);
    if( status == CY_SCB_I2C_SUCCESS )
        data->dig_P9 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    return status;
}

cy_en_scb_i2c_status_t BMP280_readValue(bmp280_calib_data cal, float* temperature, float*pressure, uint32_t timeout)
{
    cy_en_scb_i2c_status_t status;
    uint8_t buffer_i2c[4];
    int32_t adc_T; int64_t var1, var2;
    status = bmp_read_bytes(0xFA, buffer_i2c, 3, 500);
    adc_T = (uint32_t)(buffer_i2c[0]) << 16 | (uint32_t)(buffer_i2c[1]) << 8 | (uint32_t)(buffer_i2c[2]);
    adc_T >>= 4;
    var1 = ((((adc_T >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
    float T = ( ((var1 + var2) * 5 + 128) >> 8 ) ;
    *temperature =  (float)(T / 100);
    
    status = bmp_read_bytes(0xF7, buffer_i2c, 3, 500);
    adc_T = (uint32_t)(buffer_i2c[0]) << 16 | (uint32_t)(buffer_i2c[1]) << 8 | (uint32_t)(buffer_i2c[2]);
    
    adc_T >>= 4;
    var1 = ((int64_t)(var1 + var2)) - 128000;
    var2 = var1 * var1 * (int64_t)cal.dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal.dig_P3) >> 8) +
         ((var1 * (int64_t)cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal.dig_P1) >> 33;

    if (var1 == 0)
        *pressure = 0; // avoid exception caused by division by zero
    else
    {
        int64_t p = 1048576 - adc_T;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)cal.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7) << 4);
        *pressure = (float)p / 256;
    }
    return status;
}

/* [] END OF FILE */
