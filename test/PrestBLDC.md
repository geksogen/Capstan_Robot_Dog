* Загружаем конфигурацию
odrivetool restore-config my_odrive_config_v5_5010_cabastian.json
* Очистка ошибок
* Делаем калибровку мотора (AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
odrv0.axis0.requested_state = 3
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.save_configuration()
* Делаем калибровку энкодера (AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
odrv0.axis0.requested_state = 7
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.save_configuration()
odrv0.reboot()