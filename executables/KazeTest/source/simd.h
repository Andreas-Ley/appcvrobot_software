#pragma once

#ifdef BUILD_FOR_ARM
    #include "simd_neon.h"
#else
    #include "simd_sse.h"
    //#include "simd_ref.h"
#endif