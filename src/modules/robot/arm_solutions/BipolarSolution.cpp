#include "BipolarSolution.h"
#include "ActuatorCoordinates.h"

#include <fastmath.h>
#include "checksumm.h"
#include "ActuatorCoordinates.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"
//#include "Gcode.h"
//#include "SerialMessage.h"
//#include "Conveyor.h"
#include "Robot.h"
#include "StepperMotor.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define PI 3.14159265358979323846F // force to be float, do not use M_PI
//#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
#define arm_length_checksum          CHECKSUM("arm_length")

BipolarSolution::BipolarSolution(Config* config)
{
    // arm_length is the distance between the two rotational centers
    arm_length = config->value(arm_length_checksum)->by_default(110.0f)->as_number();
    
    raw_mode = false;
}

void BipolarSolution::init() {

}

void BipolarSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm )
{
	float x = cartesian_mm[X_AXIS];
	float y = cartesian_mm[Y_AXIS];
	float z = cartesian_mm[Z_AXIS];
	
	if(raw_mode)
    {
        actuator_mm[ALPHA_STEPPER] = x;
        actuator_mm[BETA_STEPPER ] = y;
        actuator_mm[GAMMA_STEPPER] = z;
    }
    else
    {
		//Convert a set of cartesian coordinates (in mm) to bipolar coordinates (in degrees).
		float theta2 = 2 * asinf(sqrtf(x*x+y*y) / (2 * arm_length) );
		float theta1 = (PI-theta2)/2 - atan2f(y,x);
		//Convert from radians to degrees
		float theta1_target = to_degrees(theta1);
		float theta2_target = to_degrees(theta2);
		//THEKERNEL->streams->printf("ok Current: %f, Target: %f\n", alpha_position, alpha_target);
        actuator_mm[ALPHA_STEPPER] = ROUND(theta1_target, 4);
        actuator_mm[BETA_STEPPER ] = ROUND(theta2_target, 4);
        actuator_mm[GAMMA_STEPPER] = z;
	}
}

void BipolarSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] )
{
	float x = actuator_mm[X_AXIS];
	float y = actuator_mm[Y_AXIS];
	float z = actuator_mm[Z_AXIS];
	
    if(raw_mode)
    {
        cartesian_mm[ALPHA_STEPPER] = x;
        cartesian_mm[BETA_STEPPER ] = y;
        cartesian_mm[GAMMA_STEPPER] = z;
    }
    else
    {
		//Convert from degrees to radians
		float theta1_rad = to_radians(x);
		float theta2_rad = to_radians(y);

		//Convert from bipolar to polar
		float theta = ((PI-theta2_rad)/2) - theta1_rad;
		float r = 2 * arm_length * sinf(theta2_rad/2);

		//Convert from polar to cartesian
		x = r * cosf(theta);
		y = r * sinf(theta);
		
		cartesian_mm[ALPHA_STEPPER] = ROUND(x, 4);
		cartesian_mm[BETA_STEPPER]  = ROUND(y, 4);
		cartesian_mm[GAMMA_STEPPER] = z;
	}
}

float BipolarSolution::to_degrees(float radians) {
    return radians * (180.0f/PI);
}

float BipolarSolution::to_radians(float degrees) {
    return degrees * (PI/180.0f);
}

bool BipolarSolution::set_optional(const arm_options_t& options)
{
    arm_options_t::const_iterator i;

    // Raw mode
    i = options.find('R');
    if(i != options.end()) {
        raw_mode = (i->second > 0.5);
    }
    return true;
}

bool BipolarSolution::get_optional(arm_options_t& options, bool force_all)
{
    if(raw_mode)
        options['R'] = 1;
    else
        options['R'] = 0;
    return true;
};
