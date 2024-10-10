#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include "fl/Headers.h"

class FuzzyController {
public:
    FuzzyController();
    float compute(float offset_value, float angle_value);

private:
    fl::Engine* engine;
};

#endif
