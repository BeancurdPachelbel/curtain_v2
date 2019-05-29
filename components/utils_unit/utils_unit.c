#include "utils_unit.h"

//初始化常量
void utils_init()
{
    task_event_group = NULL;
    TASK_BIT = BIT0;
    is_active = false;
}