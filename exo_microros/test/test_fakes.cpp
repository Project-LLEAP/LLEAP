#include "test_fakes.hpp"
#include "shared.hpp"

// Shared state implementation
uint32_t fake_ms = 0;

portMUX_TYPE g_shared_mux = portMUX_INITIALIZER_UNLOCKED;

// Create SHARED variable
SharedState SHARED; 