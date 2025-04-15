#include "driver_buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void BuzzerTest(void) {
    buzzer_init();
    
    note(NOTE_C, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_E, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_G, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_C, OCTAVE_5, 75);
    vTaskDelay(pdMS_TO_TICKS(500));
    note(NOTE_G, OCTAVE_5, 750);

    vTaskDelay(pdMS_TO_TICKS(1000));

    note(NOTE_C, OCTAVE_4, 100);
    note_transition(NOTE_E, OCTAVE_4, 50);
    note(NOTE_E, OCTAVE_4, 100);
    note_transition(NOTE_G, OCTAVE_4, 50);
    note(NOTE_G, OCTAVE_4, 100);
    note_transition(NOTE_C, OCTAVE_5, 50);
    note(NOTE_G, OCTAVE_5, 750);
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    note_t notes[] = {NOTE_C, NOTE_E, NOTE_G, NOTE_C};
    octave_t octaves[] = {OCTAVE_2, OCTAVE_2, OCTAVE_2, OCTAVE_3};
    note_t notes1[] = {NOTE_D, NOTE_F, NOTE_A, NOTE_C, NOTE_E};
    octave_t octaves1[] = {OCTAVE_1, OCTAVE_1, OCTAVE_1, OCTAVE_2, OCTAVE_2};

    note_harmonics_waveform(notes, octaves, 4, 1000); // C4+E4+G4 for 1s
    note_harmonics_waveform(notes1, octaves1, 5, 1000); // C4+E4+G4 for 1s
}
