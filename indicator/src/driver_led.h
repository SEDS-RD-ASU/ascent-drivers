#include <stdint.h>
#include <stdbool.h>

// Public macros
#define LED_PIN 21
#define RST_US 1000

// Type definitions
typedef struct { uint8_t g, r, b; } led_color_t;

// Function declarations
void led_init(void);
void send_rgb(led_color_t c);
void send_color(const char* color_name);
void send_reset(void);