#include "ina219_unit.h"

#define TAG		"INA219"

/** Register Addresses ********************************************************/
#define INA219_DEVICE_ADDRESS			0x40

#define INA219_REG_CONFIG				(0x00)				// CONF REGISTER (R/W)
#define INA219_REG_SHUNTVOLTAGE			(0x01)				// SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE			(0x02) 				// BUS VOLTAGE REGISTER (R)
#define INA219_REG_POWER				(0x03)				// POWER REGISTER (R)
#define INA219_REG_CURRENT				(0x04)				// CURRENT REGISTER (R)
#define INA219_REG_CALIBRATION			(0x05)				// CALIBRATION REGISTER (R/W)

/** Configuration Register ****************************************************/
#define INA219_CONFIG_RESET						(0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK		(0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V			(0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V			(0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK					(0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV				(0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV				(0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV				(0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV				(0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK				(0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT				(0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT				(0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT				(0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT				(0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK				(0x0078)	// Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US		(0x0000)	// 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US	(0x0008)	// 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US	(0x0010)	// 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US	(0x0018)	// 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US	(0x0048)	// 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US	(0x0050)	// 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US	(0x0058)	// 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US	(0x0060)	// 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS	(0x0068)	// 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS	(0x0070)	// 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS	(0x0078)	// 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK					(0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN			(0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED		(0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED		(0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	(0x0003)
#define INA219_CONFIG_MODE_ADCOFF				(0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS		(0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS		(0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS	(0x0007)


/************ I2C **************************/ 
#define DATA_LENGTH 					512         		// Data buffer length of test buffer
#define RW_LENGTH 						128         		// Data length for r/w test, [0,DATA_LENGTH]
#define DELAY_TIME_BETWEEN_ITEMS_MS 	1000 				// delay time between different test items

#define I2C_MASTER_SCL_IO 				22					// gpio number for I2C master clock
#define I2C_MASTER_SDA_IO 				23					// gpio number for I2C master data
#define I2C_MASTER_NUM 					1					// I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 				100000				// I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 		0           		// I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 		0           		// I2C master doesn't need buffer

#define WRITE_BIT 						I2C_MASTER_WRITE	// I2C master write
#define READ_BIT 						I2C_MASTER_READ     // I2C master read
#define ACK_CHECK_EN 					0x1                 // I2C master will check ack from slav
#define ACK_CHECK_DIS 					0x0                 // I2C master will not check ack from slave
#define ACK_VAL 						0x0                 // I2C ack value
#define NACK_VAL 						0x1                 // I2C nack value

/* Global Variables ***********************************************************/
uint8_t ina219_i2caddr;
uint32_t ina219_calValue;
// The following multipliers are used to convert raw current and power
// values to mA and mW, taking into account the current config settings
uint32_t ina219_currentDivider_mA;
uint32_t ina219_powerDivider_mW;

//I2C总线初始化
void i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    //可在此处判断esp_err
    esp_err_t err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if ( err == ESP_OK)
    {
    	ESP_LOGI(TAG, "I2C总线初始化成功");
    }
}

//设置INA219的测量参数
void ina219_setCalibration_32V_2A()
{
	ina219_calValue = 4096;
	ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_writeRegister(INA219_REG_CONFIG, config);
}

//初始化INA219
void ina219_init()
{
	i2c_init();

	ina219_currentDivider_mA = 0;
	ina219_powerDivider_mW = 0;
	vTaskDelay( 15 / portTICK_RATE_MS);

	//设置测量标准
	ina219_setCalibration_32V_2A();

	vTaskDelay(1000 / portTICK_RATE_MS);

	//读取电流任务
	xTaskCreate(read_current_task, "read_current_task", 1024*4, NULL, 6, &read_current_handle);
}

//INA219写寄存器
esp_err_t ina219_writeRegister(uint8_t reg, uint16_t data)
{
	//most significant bit
	uint8_t msb = (uint8_t)(data >> 8);
	//less significant bit
	uint8_t lsb = (uint8_t)(data & 0xff);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //write device register
    i2c_master_write_byte(cmd, (INA219_DEVICE_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    //write function register
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //write most significant bit
    i2c_master_write_byte(cmd, msb, ACK_CHECK_EN);
    //write less significant bit
    i2c_master_write_byte(cmd, lsb, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    // if ( ret == ESP_OK)
    // {
    // 	ESP_LOGI(TAG, "写寄存器成功");
    // }
    // else
    // {
    // 	ESP_LOGW(TAG, "写寄存器失败");
    // }
    i2c_cmd_link_delete(cmd);
    vTaskDelay(1/portTICK_PERIOD_MS);
    return ret;
}

//INA219读寄存器
esp_err_t ina219_readRegister(uint8_t reg, uint16_t *data)
{ 
	
	ina219_writeRegister(reg, 0);
	
	uint8_t data_h = 0, data_l = 0;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	//write device register
    i2c_master_write_byte(cmd, INA219_DEVICE_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
	//read value
    i2c_master_read_byte(cmd, &data_h, ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    // if ( ret == ESP_OK)
    // {
    // 	ESP_LOGI(TAG, "读寄存器成功");
    // }
    // else
    // {
    // 	ESP_LOGW(TAG, "读寄存器失败");
    // }
    i2c_cmd_link_delete(cmd);
    //i2c读取的数据必须在i2c命令运行完成之后才能准确读取，否则为空
    //assemble data
    *data = (data_h << 8) | data_l;
    // ESP_LOGI(TAG, "data_h:%d", data_h);
    // ESP_LOGI(TAG, "data_l:%d", data_l);

    return ret;
	
}

//读取电流(二进制值)
int16_t ina219_getCurrent_raw()
{
	uint16_t value;

	// Sometimes a sharp load will reset the INA219, which will
	// reset the cal register, meaning CURRENT and POWER will
	// not be available ... avoid this by always setting a cal
	// value even if it's an unfortunate extra step
	// 
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);

	// Now we can safely read the CURRENT register!
	ina219_readRegister(INA219_REG_CURRENT, &value);

	return (int16_t)value;
}

//读取电流(浮点值)
float ina219_getCurrent_mA()
{
	float valueDec = ina219_getCurrent_raw();
	valueDec /= ina219_currentDivider_mA;
	return valueDec;
}

//测试读取电流值任务
void read_current_task(void *arg)
{
	ESP_LOGI(TAG, "检测电流值任务");
	float current;
	float static_current_min = 55.0;
	float static_current_max = 700.0;
	vTaskDelay(2000 / portTICK_RATE_MS);
	while(1)
	{
		current = ina219_getCurrent_mA();

		// ESP_LOGI(TAG, "当前电流值为:%2.1lfmA\n", current);
		// vTaskDelay(1000 / portTICK_RATE_MS);

		//如果步进电机没有处于旋转中
		//判断is_runnable，如果可以则删除is_just_running
		if (!get_is_just_running())
		{
			if (current > static_current_min)
			{
				ESP_LOGI(TAG, "当前手拉电流值为:%2.1lfmA\n", current);
				//电机反转
				stepper_reverse();
			}
		}

		if (static_current_max < current)
		{
			ESP_LOGI(TAG, "当前电流值为:%2.1lfmA\n", current);
			stop_running();
		}
		vTaskDelay(20 / portTICK_RATE_MS);
	}
}

