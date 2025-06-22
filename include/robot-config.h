using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller controller1;
extern motor fl;
extern motor ml;
extern motor bl;
extern motor fr;
extern motor mr;
extern motor br;
extern motor intake;
extern inertial inertialSensor;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );