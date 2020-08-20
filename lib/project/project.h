// **** Project code definition here ...
#include <ambient.h>



// **** Project code functions here ...
void project_hw() {
 // Output GPIOs


 // Input GPIOs

}


void project_setup() {
  // Start Ambient devices
      ambient_setup();
      //TIMER = 15;                                       // TIMER value (Recommended 15 minutes) to get Ambient data.

}

void project_loop() {
  // Ambient handing
      if (TIMER >0) if ((millis() - 3500) % (TIMER * 60000) < 5) ambient_data();      // TIMER bigger than zero on div or dog bites!!

}
