/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_ObserverbasedAdmittance_sample_controller.h"

MULTI_CONTROLLERS_CONSTRUCTOR(
    "ObserverbasedAdmittanceSample",
    ObserverbasedAdmittanceSampleController(rm, dt, config, mc_control::MCController::Backend::Tasks),
    "ObserverbasedAdmittanceSample_TVM",
    ObserverbasedAdmittanceSampleController(rm, dt, config, mc_control::MCController::Backend::TVM))
