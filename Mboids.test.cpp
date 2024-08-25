#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "Boids.hpp"
#include "doctest.h"

TEST_CASE("Testing abs_distance_from") {
  SUBCASE("Testing abs_distance_from of a stationary boid from itself") {
    std::vector<Boid> flock;
    Boid boid_i({201., 201.}, {0., 0.});
    flock.push_back(boid_i);
    float abs_dist = boid_i.abs_distance_from(boid_i);
    CHECK(abs_dist == 0);
  }

  SUBCASE("Testing abs_distance_from between two moving boids") {
    std::vector<Boid> flock;
    Boid boid_i({208., 203.}, {5., 6.});
    Boid boid_j({204., 206.}, {3., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    double abs_dist = boid_i.abs_distance_from(boid_j);
    CHECK(abs_dist == 5.);
  }

  SUBCASE(
      "Testing abs_distance_from between two moving boids, one with zero "
      "values "
      "of position") {
    std::vector<Boid> flock;
    Boid boid_i({0., 0.}, {2., 3.});
    Boid boid_j({203., 204.}, {7., 5.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    double abs_dist = boid_i.abs_distance_from(boid_j);
    CHECK(abs_dist == doctest::Approx(287.793).epsilon(0.001));
  }
}

TEST_CASE(
    "Testing update function indirectly testing separation with three "
    "boids") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200., 203.}, {1.5, 2.5});
  Boid boid_j({210., 202.}, {3., 1.});
  Boid boid_m({204., 215.}, {1., 2.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.5, 0., 0., 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == -5.5);
  CHECK(flock[0].getVelocity().y == -3.);
  CHECK(flock[0].getPosition().x == 194.5);
  CHECK(flock[0].getPosition().y == 200);
  CHECK(flock[1].getVelocity().x == 13.75);
  CHECK(flock[1].getVelocity().y == -4.5);
  CHECK(flock[1].getPosition().x == 223.75);
  CHECK(flock[1].getPosition().y == 197.5);
  CHECK(flock[2].getVelocity().x == -4.125);
  CHECK(flock[2].getVelocity().y == 18.25);
  CHECK(flock[2].getPosition().x == 199.875);
  CHECK(flock[2].getPosition().y == 233.25);
}

TEST_CASE(
    "Testing update function indirectly testing alignment with three boids") {
  std::vector<Boid> flock;
  const float max_speed = 10;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({300., 303.}, {2., 1.5});
  Boid boid_j({305., 302.}, {3., 2.5});
  Boid boid_m({304., 295.}, {1., 2.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0., 0.5, 0., 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 2.);
  CHECK(flock[0].getVelocity().y == 1.875);
  CHECK(flock[0].getPosition().x == 302.);
  CHECK(flock[0].getPosition().y == 304.875);
  CHECK(flock[1].getVelocity().x == 2.25);
  CHECK(flock[1].getVelocity().y == doctest::Approx(2.219).epsilon(0.001));
  CHECK(flock[1].getPosition().x == 307.25);
  CHECK(flock[1].getPosition().y == doctest::Approx(304.219).epsilon(0.001));
  CHECK(flock[2].getVelocity().x == doctest::Approx(1.563).epsilon(0.001));
  CHECK(flock[2].getVelocity().y == doctest::Approx(2.023).epsilon(0.001));
  CHECK(flock[2].getPosition().x == doctest::Approx(305.563).epsilon(0.001));
  CHECK(flock[2].getPosition().y == doctest::Approx(297.023).epsilon(0.001));
}

TEST_CASE(
    "Testing update function indirectly testing cohesion with three boids") {
  std::vector<Boid> flock;
  const float max_speed = 10;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({401., 724.}, {3., 1.});
  Boid boid_j({404., 726.}, {-5., 2.});
  Boid boid_m({399., 723.}, {2., -4.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0., 0., 0.5, 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 3.25);
  CHECK(flock[0].getVelocity().y == 1.25);
  CHECK(flock[0].getPosition().x == 404.25);
  CHECK(flock[0].getPosition().y == 725.25);
  CHECK(flock[1].getVelocity().x == doctest::Approx(-6.189).epsilon(0.001));
  CHECK(flock[1].getVelocity().y == doctest::Approx(1.063).epsilon(0.001));
  CHECK(flock[1].getPosition().x == doctest::Approx(397.811).epsilon(0.001));
  CHECK(flock[1].getPosition().y == doctest::Approx(727.063).epsilon(0.001));
  CHECK(flock[2].getVelocity().x == doctest::Approx(3.015).epsilon(0.001));
  CHECK(flock[2].getVelocity().y == doctest::Approx(-2.423).epsilon(0.001));
  CHECK(flock[2].getPosition().x == doctest::Approx(402.015).epsilon(0.001));
  CHECK(flock[2].getPosition().y == doctest::Approx(720.578).epsilon(0.001));
}

/*
TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({500., 856.}, {1., 7.});
  Boid boid_j({502., 854.}, {5., 6.});
  Boid boid_m({490., 859.}, {4., 4.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6, 0.4, 0.5, 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}

TEST_CASE("Testing update function indirectly testing avoid_edges") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({1821., 99.}, {2.5, 2.5});
  Boid boid_j({1820., 100.}, {1., 1.});
  Boid boid_m({1819, 101}, {1., 1.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0., 0., 0., 3., 3.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}

TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together with avoid_edges") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({99., 981.}, {5., 7.});
  Boid boid_j({102., 980.}, {7., 4.});
  Boid boid_m({108., 979.}, {8., 6.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6, 0.4, 0.5, 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}

TEST_CASE("Testing update function indirectly testing limit") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200., 205.}, {25., 30.});
  Boid boid_j({201., 204.}, {22., 27.});
  Boid boid_m({203., 201.}, {31., 28.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0., 0., 0., 3., 3.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}

TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together with limit") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200., 640.}, {20., 25.});
  Boid boid_j({201., 644.}, {21., 30.});
  Boid boid_m({208., 638.}, {19., 26.});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6, 0.4, 0.5, 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}

TEST_CASE(
    "Testing update function indirecty testing separation and alignment and "
    "cohesion with reiteration") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({405., 201.}, {3.5, 2.5});
  Boid boid_j({401., 207.}, {1., 2.});
  Boid boid_m({403., 203.}, {3., 1.5});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.5, 2., 2., 3., 3.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 0.);
  CHECK(flock[0].getVelocity().y == 0.);
  CHECK(flock[0].getPosition().x == 0.);
  CHECK(flock[0].getPosition().y == 0.);
  CHECK(flock[1].getVelocity().x == 0.);
  CHECK(flock[1].getVelocity().y == 0.);
  CHECK(flock[1].getPosition().x == 0.);
  CHECK(flock[1].getPosition().y == 0.);
  CHECK(flock[2].getVelocity().x == 0.);
  CHECK(flock[2].getVelocity().y == 0.);
  CHECK(flock[2].getPosition().x == 0.);
  CHECK(flock[2].getPosition().y == 0.);
}
*/

TEST_CASE(
    "Testing update function indirectly testing separation and alignment and "
    "cohesion with one single boid") {
  std::vector<Boid> flock;
  const float max_speed = 20;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200., 201.}, {5., 8.});
  flock.push_back(boid_i);

  Params params{0.6, 0.4, 0.5, 35., 30.};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 5.);
  CHECK(flock[0].getVelocity().y == 8.);
  CHECK(flock[0].getPosition().x == 205.);
  CHECK(flock[0].getPosition().y == 209.);
}

TEST_CASE("Testing v_mean") {
  SUBCASE("Testing v_mean with three boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {2.5, 2.5});
    Boid boid_j({101., 101.}, {1.5, 1.5});
    Boid boid_m({104., 104.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.v_mean == doctest::Approx(2.828).epsilon(0.001));
  }

  SUBCASE(
      "Testing v_mean with three boids, one with negative values of "
      "velocity") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {-0.5, -0.5});
    Boid boid_j({101., 101.}, {1.5, 1.5});
    Boid boid_m({104., 104.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.v_mean == doctest::Approx(1.414).epsilon(0.001));
  }

  SUBCASE("Testing v_mean with one single boid") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {2.5, 2.5});
    flock.push_back(boid_i);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.v_mean == doctest::Approx(3.536).epsilon(0.001));
  }

  SUBCASE("Testing v_mean with values of velocity that cancel each other out") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {-2.5, -2.5});
    Boid boid_j({101., 101.}, {1.5, 1.5});
    Boid boid_m({104., 104.}, {1., 1.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.v_mean == doctest::Approx(0.).epsilon(0.001));
  }
}

TEST_CASE("Testing d_mean") {
  SUBCASE("Testing d_mean with three boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({202., 202.}, {2.5, 2.5});
    Boid boid_j({201., 201.}, {1.5, 1.5});
    Boid boid_m({203., 203.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.d_mean == doctest::Approx(1.886).epsilon(0.001));
  }

  SUBCASE("Testing d_mean with more distant boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({202., 202.}, {2.5, 2.5});
    Boid boid_j({450., 450.}, {1.5, 1.5});
    Boid boid_m({978., 978.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.d_mean == doctest::Approx(731.620).epsilon(0.001));
  }
}

TEST_CASE("Testing sigma_v") {
  SUBCASE("Testing sigma_v with three boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {2.5, 2.5});
    Boid boid_j({101., 101.}, {1.5, 1.5});
    Boid boid_m({104., 104.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.sigma_v == doctest::Approx(0.707).epsilon(0.001));
  }

  SUBCASE(
      "Testing sigma_v with three boids, one with negative values of "
      "velocity") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({102., 102.}, {-0.5, -0.5});
    Boid boid_j({101., 101.}, {1.5, 1.5});
    Boid boid_m({104., 104.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.sigma_v == doctest::Approx(1.225).epsilon(0.001));
  }
}

TEST_CASE("Testing sigma_d") {
  SUBCASE("Testing sigma_d with three boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({202., 202.}, {2.5, 2.5});
    Boid boid_j({201., 201.}, {1.5, 1.5});
    Boid boid_m({203., 203.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.sigma_d == doctest::Approx(0.943).epsilon(0.001));
  }
  SUBCASE("Testing sigma_d with more distant boids") {
    std::vector<Boid> flock;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    Stats stats{};
    Boid boid_i({202., 202.}, {2.5, 2.5});
    Boid boid_j({450., 450.}, {1.5, 1.5});
    Boid boid_m({978., 978.}, {2., 2.});
    flock.push_back(boid_i);
    flock.push_back(boid_j);
    flock.push_back(boid_m);
    stats = calculateStatistics(flock, start_time);
    CHECK(stats.sigma_d == doctest::Approx(431.374).epsilon(0.001));
  }
}