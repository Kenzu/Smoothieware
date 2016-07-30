#ifndef BIPOLARSOLUTION_H
#define BIPOLARSOLUTION_H
#include "BaseSolution.h"

class Config;

class BipolarSolution : public BaseSolution {
    public:
        BipolarSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) override;

        bool set_optional(const arm_options_t& options) override;
        bool get_optional(arm_options_t& options, bool force_all) override;

    private:
        void init();
        float arm_length;
        float to_degrees(float radians);
        float to_radians(float degrees);
        bool raw_mode;
};

#endif // BIPOLARSOLUTION_H
