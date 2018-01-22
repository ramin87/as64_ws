#ifndef DMP_UTILS_H
#define DMP_UTILS_H

#include <DMP_lib/CanonicalClock/CanonicalClock.h>
#include <DMP_lib/CanonicalClock/LinCanonicalClock.h>

#include <DMP_lib/GatingFunction/GatingFunction.h>
#include <DMP_lib/GatingFunction/LinGatingFunction.h>
#include <DMP_lib/GatingFunction/ExpGatingFunction.h>
#include <DMP_lib/GatingFunction/ConstGatingFunction.h>
#include <DMP_lib/GatingFunction/SigmoidGatingFunction.h>
#include <DMP_lib/GatingFunction/SpringDamperGatingFunction.h>

#include <DMP_lib/DMP/DMP_.h>
#include <DMP_lib/DMP/DMP.h>
#include <DMP_lib/DMP/DMP_bio.h>
#include <DMP_lib/DMP/DMP_plus.h>
#include <DMP_lib/DMP/DMP_Shannon.h>

namespace as64_
{

std::shared_ptr<CanonicalClock> getCanClock(const std::string &CAN_CLOCK_TYPE, double tau);


std::shared_ptr<DMP_> getDMP(const std::string &DMP_TYPE, int N_kernels, double a_z, double b_z,
                            std::shared_ptr<CanonicalClock> canClockPtr,
                            std::shared_ptr<GatingFunction> shapeAttrGatingPtr,
                            std::shared_ptr<GatingFunction> goalAttrGatingPtr,
                            const param_::ParamList *paramListPtr=NULL);


std::shared_ptr<GatingFunction> getGatingFun(const std::string &GATTING_FUN_TYPE, double u0, double u_end);

} // namespace as64_

#endif // DMP_UTILS_H
