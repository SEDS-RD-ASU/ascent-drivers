#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_lora.h"

#define NUM_BYTES 254  // Change this to modify number of bytes to send

void task_tx(void *p)
{
    uint8_t data[NUM_BYTES];
    
    for(;;) {
        // Fill the data array with increasing numbers
        for (int j = 0; j < NUM_BYTES; j++) {
            data[j] = (j + 1) % 254;  // Wrap around at 256
        }
        
        // Send the packet
        lora_send_packet(data, NUM_BYTES);
        printf("Sent packet\n");
    }
}

void app_main()
{
   lora_init();

   lora_set_bandwidth(125E3); // I've had issues with higher bandwidths, possibly due to having no antenna
   lora_disable_crc(); // THIS WILL DISABLE ERROR CORRECTION!!!

   // Impact on signal range for these settings below is dubious, wire an antenna up to test
   lora_set_coding_rate(5);
   lora_set_spreading_factor(6);

   // These set a sync word, make sure the reciever has these two lines in the init code
   lora_set_sync_word(0x12);  
   lora_set_frequency(915e6);
   
   xTaskCreate(&task_tx, "task_tx", 4096, NULL, 5, NULL);
}
