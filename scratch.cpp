#include <Arduino.h>
#include <Bluepad32.h>
void test(ControllerPtr ctl) {
    ControllerProperties props = ctl->getProperties();
    String model = ctl->getModelName();
}
