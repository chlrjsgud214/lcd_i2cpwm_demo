add_executable(
  ${PRJ_NAME}
  
  main.c
  view/lcd_run.c
  lib/config/DEV_config.c
  lib/fatfs/diskio.c
  lib/fatfs/fatfs_storage.c
  lib/fatfs/ff.c
  lib/font/font8.c
  lib/font/font12.c
  lib/font/font16.c
  lib/font/font20.c
  lib/font/font24.c
  lib/lcd/LCD_Bmp.c
  lib/lcd/LCD_Driver.c  
  lib/lcd/LCD_GUI.c
  lib/lcd/LCD_Touch.c
  lib/sdcard/MMC_SD.c
  lib/PCA9685/pca9685.c
  
  
  

)

target_include_directories(${PRJ_NAME} PUBLIC 
    lib/config
    lib/fatfs
    lib/font
    lib/lcd
    lib/sdcard
    lib/pca9685
    
    view
)

target_link_libraries(${PRJ_NAME} 
  cmsis_core
  pico_stdlib
  hardware_spi
  hardware_dma
  hardware_irq
  hardware_i2c
  hardware_flash
  hardware_pwm
  hardware_pio
  )

pico_add_extra_outputs(${PRJ_NAME})
pico_enable_stdio_usb(${PRJ_NAME}  1)
pico_enable_stdio_uart(${PRJ_NAME} 0)

set_target_properties(${PRJ_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${PRJ_NAME} POST_BUILD
        COMMAND arm-none-eabi-size --format=berkeley --totals "$<TARGET_FILE:${PRJ_NAME}>"
        COMMENT "Invoking: Cross ARM GNU Print Size")