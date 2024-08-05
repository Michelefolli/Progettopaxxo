#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "Boids_ver_2.hpp"
#include "doctest.h"

/*TEST_CASE("Testing median stats") {
  Sim sim;
  sim.add({4, 4, 1, -1});
  sim.add({0, 1, 1, -1});
  CHECK(sim.statistics().v_media == sqrt(2));
  CHECK(sim.statistics().d_media==5);

}

TEST_CASE("Testing abs distance"){
  Boid A {1,1,0,0};
  CHECK(abs_distance(A, A)==0);
}
*/
TEST_CASE("Testing travel function") {
  Sim sim;
  Boid boid(Vec_2d(4., 4.), Vec_2d(1., 1.));
  Boid boid_j({5., 5.}, {2, 2});
  sim.add(boid);
  sim.add(boid_j);
  sim.travel();
  CHECK(sim.stormo_[0].position.x == 5);
}

TEST_CASE("Testing separation function") {
  Sim sim;
  Boid boid_i({2, 2}, {2.5, 2.5});
  Boid boid_j({1, 1}, {1., 1.});
  Boid boid_m({4, 4}, {1., 1.});
  sim.add(boid_i);
  sim.add(boid_j);
  sim.add(boid_m);
  sim.GetParams(0.5, 2, 2, 3., 3.);
  sim.alignment_and_cohesion();
  CHECK(sim.stormo_[0].velocity.x == 0.5);
  CHECK(sim.stormo_[0].velocity.y == 0.5);
}

TEST_CASE("Testing separation function") {
  Sim sim;
  Boid boid_i({2, 2}, {2.5, 2.5});
  Boid boid_j({1, 1}, {1., 1.});
  Boid boid_m({4, 4}, {1., 1.});
  sim.add(boid_i);
  sim.add(boid_j);
  sim.add(boid_m);
  sim.GetParams(0.5, 2, 2, 3., 3.);
  sim.separation();
  CHECK(sim.stormo_[0].velocity.x == 2);
  CHECK(sim.stormo_[0].velocity.y == 2);
}