#ifndef EXTERNAL_LIBS_H
#define EXTERNAL_LIBS_H

// --- IMPORTANT ---
// Include PyTorch (libtorch) headers BEFORE CasADi headers to resolve
// the operator<< conflict between glog (used by Torch) and CasADi.
#include "external_libs.h"

// Now, include the CasADi header
#include "external_libs.h"

#endif // EXTERNAL_LIBS_H