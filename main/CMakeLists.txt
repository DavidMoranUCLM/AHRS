idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gsl 
                             buffer
                             EKF_C
                             mpu9250
                             esp_partition
                             esp_timer
                             partitionLog)
                             
