#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "Mboids.hpp"
#include "doctest.h"

TEST_CASE("Testing median stats") {
  Sim sim;
  sim.add({4, 4, 1, -1});
  sim.add({0, 1, 1, -1});
  CHECK(sim.statistics().v_media == sqrt(2));
  CHECK(sim.statistics().d_media==5);
}