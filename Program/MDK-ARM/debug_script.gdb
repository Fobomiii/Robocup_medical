target extended-remote localhost:3333
monitor reset halt
monitor reset init

# 设置断点在 UART8 接收回调
break HAL_UART_RxCpltCallback

# 监控 NUC 障碍物数据
display nuc_obstacle_data
display nuc_obstacle_valid
display nuc_last_update_tick

# 继续运行
continue
