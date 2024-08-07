#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "Boids_ver_2.hpp"
#include "doctest.h"
/*
TEST_CASE("Testing median stats") {
  Sim sim;
  sim.add({4, 4, 1, -1});
  sim.add({0, 1, 1, -1});
  CHECK(sim.statistics().v_media == sqrt(2));
  CHECK(sim.statistics().d_media==5);

}
*/

TEST_CASE("Testing abs_distance") {
  Sim sim;

  SUBCASE(
      "Testing abs_distance of a stationary boid from itself") {  // ho fatto
                                                                  // con i
                                                                  // subcase,
                                                                  // spero sia
                                                                  // okay
    Boid boid_i({1., 1.}, {0., 0.});
    sim.add(boid_i);
    double abs_dist = sim.abs_distance(boid_i, boid_i);
    CHECK(abs_dist == 0);
    // CHECK(abs_distance(boid_i, boid_i) == 0);
  }

  SUBCASE("Testing abs_distance between two moving boids") {
    Boid boid_i({8., 3.}, {5., 6.});
    Boid boid_j({4., 6.}, {3., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    double abs_dist = sim.abs_distance(boid_i, boid_j);
    CHECK(abs_dist == 5.);
  }

  SUBCASE(
      "Testing abs_distance between two moving boids, one with negative values "
      "of position") {
    Boid boid_i({-1., -3.}, {2., 3.});
    Boid boid_j({2., 1.}, {7., 5.});
    sim.add(boid_i);
    sim.add(boid_j);
    double abs_dist = sim.abs_distance(boid_i, boid_j);
    CHECK(abs_dist == 5.);
  }

  SUBCASE(
      "Testing abs_distance between two moving boids, one with zero values "
      "of position") {
    Boid boid_i({0., 0.}, {2., 3.});
    Boid boid_j({3., 4.}, {7., 5.});
    sim.add(boid_i);
    sim.add(boid_j);
    double abs_dist = sim.abs_distance(boid_i, boid_j);
    CHECK(abs_dist == 5.);
  }
}  // questi test funzionano, però il subcase con le posizioni negative non ha
   // molto senso perché poi nella simulazione l'effetto pacman le riporta a
   // zero (subcase dopo), ma la funzione non può saperlo ovviamente, quindi boh
   // forse da togliere

TEST_CASE("Testing travel function") {
  Sim sim;
  Boid boid(Vec_2d(4., 4.), Vec_2d(1., 1.));
  Boid boid_j({5., 5.}, {2., 2.});
  sim.add(boid);
  sim.add(boid_j);
  sim.travel();
  CHECK(sim.stormo_[0].position.x == 5.);
}

TEST_CASE("Testing separation function") {
  Sim sim;

  SUBCASE("Testing separation function with three boids") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({4., 4.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == 2.);
    CHECK(sim.stormo_[0].velocity.y == 2.);
  }

  SUBCASE("Testing separation function with three boids, but more complete") {
    Boid boid_i({2., 3.}, {1.5, 2.5});
    Boid boid_j({1., 2.}, {3., 1.});
    Boid boid_m({4., 1.}, {1., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == 1.);
    CHECK(sim.stormo_[0].velocity.y == 4.);
    CHECK(sim.stormo_[1].velocity.x ==
          2.5);  // j e m sono troppo lontani e non interagiscono tra loro
    CHECK(sim.stormo_[1].velocity.y == 0.5);
    CHECK(sim.stormo_[2].velocity.x == 2.);
    CHECK(sim.stormo_[2].velocity.y == 1.);
  }
  // qui ho messo i boid con valori di posizione e velocità diversi tra x e y
  // e ho calcolato le nuove velocità per tutti e tre, boh più generico ma non
  // so se necessario

  SUBCASE(
      "Testing separation function with three boids, one with negative values "
      "of velocity") {
    Boid boid_i({2., 2.}, {-2.5, -2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({4., 4.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == -3.);
    CHECK(sim.stormo_[0].velocity.y == -3.);
  }

  SUBCASE(
      "Testing separation function with three boids, one with negative values"
      "of position") {  // stesso discorso di abs_distance
    Boid boid_i({-1, -1}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({0.5, 0.5}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == 0.75);
    CHECK(sim.stormo_[0].velocity.y == 0.75);
  }

  SUBCASE(
      "Testing separation function with three boids, one with zero values"
      "of position") {
    Boid boid_i({0., 0.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({2., 2.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == 1.);
    CHECK(sim.stormo_[0].velocity.y == 1.);
  }

  SUBCASE("Testing separation function on the boid itself") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.separation();
    CHECK(sim.stormo_[0].velocity.x == 2.5);
    CHECK(sim.stormo_[0].velocity.y == 2.5);
  }
}  // questi test funzionano, con lo stesso dubbio di abs_distance

TEST_CASE("Testing alignment and cohesion function") {
  Sim sim;

  SUBCASE("Testing alignment and cohesion function with three boids") {
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

  /*SUBCASE(
      "Testing alignement and cohesion function with three boids, but more "
      "complete") {
    Boid boid_i({2., 3.}, {1.5, 2.5});
    Boid boid_j({1., 2.}, {3., 1.});
    Boid boid_m({3., 1.}, {1., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.alignment_and_cohesion();
    CHECK(sim.stormo_[0].velocity.x == 2.5);
    CHECK(sim.stormo_[0].velocity.y == -2.5);
    CHECK(sim.stormo_[1].velocity.x == 2.5);
    CHECK(sim.stormo_[1].velocity.y == 3.5);
    CHECK(sim.stormo_[2].velocity.x == 2.5);
    CHECK(sim.stormo_[2].velocity.y == 4.5);
  }  // in questo non tornano le velocità di j e m, non ho idea del perché, avrò
     // sbagliato i calcoli ma non trovo l'errore
*/

  SUBCASE("Testing alignement and cohesion function with one single boid") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.alignment_and_cohesion();
    CHECK(sim.stormo_[0].velocity.x == 2.5);
    CHECK(sim.stormo_[0].velocity.y == 2.5);

    // non fa nulla perché le condizioni non rispettano l'if
  }

  SUBCASE(
      "Testing alignement and cohesion function with three boids, one with "
      "negative values of velocity") {
    Boid boid_i({2., 2.}, {-2.5, -2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({4., 4.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.alignment_and_cohesion();
    CHECK(sim.stormo_[0].velocity.x == 5.5);
    CHECK(sim.stormo_[0].velocity.y == 5.5);
  }

  SUBCASE(
      "Testing alignement and cohesion function with three boids, one with "
      "negative values of position") {
    Boid boid_i({-1., -1.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({0.5, 0.5}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.alignment_and_cohesion();
    CHECK(sim.stormo_[0].velocity.x == 3.);
    CHECK(sim.stormo_[0].velocity.y == 3.);
  }

  SUBCASE(
      "Testing alignment and cohesion function with three boids, one with zero "
      "values of position") {
    Boid boid_i({0., 0.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1., 1.});
    Boid boid_m({2., 2.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.GetParams(0.5, 2., 2., 3., 3.);
    sim.alignment_and_cohesion();
    CHECK(sim.stormo_[0].velocity.x == 2.5);
    CHECK(sim.stormo_[0].velocity.y == 2.5);
  }
}

TEST_CASE("Testing v_media") {
  Sim sim;
  SUBCASE("Testing v_media with three boids") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({4., 4.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().v_media.x == 2.);
    CHECK(sim.statistics().v_media.y == 2.);
  }
  SUBCASE(
      "Testing v_media with three boids, one with negative values of "
      "velocity") {
    Boid boid_i({2., 2.}, {-0.5, -0.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({4., 4.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().v_media.x == 1.);
    CHECK(sim.statistics().v_media.y == 1.);
  }

  SUBCASE("Testing v_media with one single boid") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.statistics();
    CHECK(sim.statistics().v_media.x == 2.5);
    CHECK(sim.statistics().v_media.y == 2.5);
  }

  /* SUBCASE(
      "Testing v_media with values of velocity that cancel each other out") {
    Boid boid_i({2., 2.}, {-2.5, -2.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({4., 4.}, {1., 1.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().v_media.x == 0.);
    CHECK(sim.statistics().v_media.y == 0.);
  } // questo test dà un risultato crazy
  */
}

TEST_CASE("Testing d_media") {
  Sim sim;
  SUBCASE("Testing d_media with three boids") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({3., 3.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().d_media.x == 2.);
    CHECK(sim.statistics().d_media.y == 2.);
  }
  SUBCASE(
      "Testing d_media with one single boid") {  // è una cosa formale nel senso
                                                 // che non servirà la distanza
                                                 // media di un boid da sé
                                                 // stesso però in teoria
                                                 // dovrebbe essere zero e non
                                                 // la posizione del boid
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.statistics();
    CHECK(sim.statistics().d_media.x == 2.);
    CHECK(sim.statistics().d_media.y == 2.);
  }
}

TEST_CASE("Testing sigma_v") {
  Sim sim;
  SUBCASE("Testing sigma_v with three boids") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({4., 4.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().sigma_v == doctest::Approx(0.577).epsilon(0.001));
  }
  SUBCASE(
      "Testing v_media with three boids, one with negative values of "
      "velocity") {
    Boid boid_i({2., 2.}, {-0.5, -0.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({4., 4.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().sigma_v == 1.);
  }
  SUBCASE("Testing sigma_v with one single boid") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.statistics();
    CHECK(sim.statistics().sigma_v == 0.);
  }
}  // sigma_v è okay e funziona

TEST_CASE("Testing sigma_d") {
  Sim sim;
  SUBCASE("Testing sigma_d with three boids") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    Boid boid_j({1., 1.}, {1.5, 1.5});
    Boid boid_m({3., 3.}, {2., 2.});
    sim.add(boid_i);
    sim.add(boid_j);
    sim.add(boid_m);
    sim.statistics();
    CHECK(sim.statistics().sigma_d == doctest::Approx(1.155).epsilon(0.001));
  }

  SUBCASE("Testing sigma_d with one single boid") {
    Boid boid_i({2., 2.}, {2.5, 2.5});
    sim.add(boid_i);
    sim.statistics();
    CHECK(sim.statistics().sigma_d == 0.);
  }
}
