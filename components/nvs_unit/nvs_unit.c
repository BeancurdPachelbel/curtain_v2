#include "nvs_unit.h"

#define  TAG	"NVS"

//NVS句柄
nvs_handle my_handle;
//NVS错误类型
esp_err_t my_err;
//存储的扇区名称
const char *sector = "storage";
//存储的键值对名称
const char *key_name = "stepper";

int stepper_count_instance = 0;


/**
 * @brief       Save the whole curtain track travel stepper count
 */
void save_stepper_count(int count)
{
	ESP_LOGI(TAG, "Saving the stepper count:%d", count);
	nvs_open(sector, NVS_READWRITE, &my_handle);
	nvs_set_i32(my_handle, key_name, count);
	nvs_commit(my_handle);
	nvs_close(my_handle);
	stepper_count_instance = count;
}


/**
 * @brief       Read the whole curtain track travel stepper count
 */
int read_stepper_count()
{
	int count = 0;
	nvs_open("storage", NVS_READWRITE, &my_handle);
	nvs_get_i32(my_handle, key_name, &count);
	nvs_close(my_handle);
	ESP_LOGI(TAG, "Reading the stepper count:%d", count);
	stepper_count_instance = count;
	return count;
}


int get_stepper_count_instance()
{
	return stepper_count_instance;
}
