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

TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({500.f, 856.f}, {1.f, -7.f});
  Boid boid_j({502.f, 854.f}, {5.f, 6.f});
  Boid boid_m({495.f, 859.f}, {-4.f, 4.f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6f, 0.4f, 0.5f, 35.f, 30.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(1.85f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(-2.55f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(501.85f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(853.45f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(5.073f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(2.333f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(507.073f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(856.333f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(-7.638f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(5.233f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(487.362f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(864.233f).epsilon(0.001f));
}

TEST_CASE("Testing update function indirectly testing avoid_edges") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({1821.f, 99.f}, {2.5f, 0.5f});
  Boid boid_j({1820.f, 100.f}, {1.f, 1.5f});
  Boid boid_m({1819.f, 101.f}, {2.f, 1.f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.f, 0.f, 0.f, 3.f, 3.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(1.692f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(1.308f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(1822.692f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(100.3f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(1.f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(1.5f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(1821.f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(101.5f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(2.f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(1.f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(1821.f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(102.f).epsilon(0.001f));
}

TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together with avoid_edges") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({99.f, 981.f}, {2.f, -5.f});
  Boid boid_j({102.f, 980.f}, {1.f, 4.f});
  Boid boid_m({108.f, 979.f}, {3.f, 0.5f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6f, 0.4f, 0.5f, 35.f, 30.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(-1.427f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(-1.823f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(97.573f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(979.177f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(0.364f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(2.774f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(102.364f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(982.774f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(7.210f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(-0.893f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(115.210f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(978.107f).epsilon(0.001f));
}

TEST_CASE("Testing update function indirectly testing limit") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200.f, 205.f}, {25.f, 30.f});
  Boid boid_j({201.f, 204.f}, {22.f, 27.f});
  Boid boid_m({203.f, 201.f}, {31.f, 28.f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.f, 0.f, 0.f, 35.f, 30.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(6.402f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(7.682f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(206.402f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(212.682f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(6.317f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(7.752f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(207.317f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(211.752f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(7.421f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(6.703f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(210.421f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(207.703f).epsilon(0.001f));
}

TEST_CASE(
    "Testing update function indirectly testing separation, alignment and "
    "cohesion, all together with limit") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({200.f, 640.f}, {20.f, 15.f});
  Boid boid_j({201.f, 644.f}, {21.f, 11.f});
  Boid boid_m({208.f, 638.f}, {19.f, 13.f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6f, 0.4f, 0.5f, 35.f, 30.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(7.895f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(6.138f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(207.895f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(646.138f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(7.440f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(6.682f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(208.440f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(650.682f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(9.778f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(2.097f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(217.778f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(640.097f).epsilon(0.001f));
}

TEST_CASE(
    "Testing update function indirecty testing separation and alignment and "
    "cohesion with reiteration and limit") {
  std::vector<Boid> flock;
  const float max_speed = 10.f;
  const float width = static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  Boid boid_i({405.f, 201.f}, {3.5f, 2.5f});
  Boid boid_j({401.f, 207.f}, {1.f, 2.f});
  Boid boid_m({403.f, 203.f}, {3.f, 1.5f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_m);
  Params params{0.6f, 0.4f, 0.5f, 46.f, 45.f};

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == 5.);
  CHECK(flock[0].getVelocity().y == doctest::Approx(-0.6f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(410.f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(200.4f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(-1.65f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(5.09f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(399.35f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(212.09f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(1.297f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(-0.473f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(404.297f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(202.527f).epsilon(0.001f));

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(8.653f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(-4.274f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(418.653f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(196.126f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(-5.614f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(8.276f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(393.736f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(220.366f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(0.058f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(-3.485f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(404.355f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(199.045f).epsilon(0.001f));

  for (auto& boid : flock) {
    boid.update(params, flock, max_speed, width, height);
  }

  CHECK(flock[0].getVelocity().x == doctest::Approx(8.484f).epsilon(0.001f));
  CHECK(flock[0].getVelocity().y == doctest::Approx(-5.294f).epsilon(0.001f));
  CHECK(flock[0].getPosition().x == doctest::Approx(427.137f).epsilon(0.001f));
  CHECK(flock[0].getPosition().y == doctest::Approx(190.832f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().x == doctest::Approx(-6.305f).epsilon(0.001f));
  CHECK(flock[1].getVelocity().y == doctest::Approx(7.762f).epsilon(0.001f));
  CHECK(flock[1].getPosition().x == doctest::Approx(387.431f).epsilon(0.001f));
  CHECK(flock[1].getPosition().y == doctest::Approx(228.128f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().x == doctest::Approx(-1.580f).epsilon(0.001f));
  CHECK(flock[2].getVelocity().y == doctest::Approx(-8.902f).epsilon(0.001f));
  CHECK(flock[2].getPosition().x == doctest::Approx(402.775f).epsilon(0.001f));
  CHECK(flock[2].getPosition().y == doctest::Approx(190.143f).epsilon(0.001f));
}

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

TEST_CASE("Testing d_mean with five distant boids") {
  std::vector<Boid> flock;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  Stats stats{};
  Boid boid_i({771.54f, 499.42f}, {5.92f, 9.07f});
  Boid boid_j({1773.37f, 247.55f}, {-2.03f, 6.59f});
  Boid boid_k({1281.38f, 763.62f}, {-6.02f, -8.50f});
  Boid boid_l({997.70f, 451.10f}, {-9.92f, -1.23f});
  Boid boid_m({950.03f, 609.44f}, {-8.56f, -8.77f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_k);
  flock.push_back(boid_l);
  flock.push_back(boid_m);
  stats = calculateStatistics(flock, start_time);
  CHECK(stats.d_mean == doctest::Approx(541.537).epsilon(0.001f));
}

TEST_CASE("Testing sigma_d with five distant boids") {
  std::vector<Boid> flock;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  Stats stats{};
  Boid boid_i({771.54f, 499.42f}, {5.92f, 9.07f});
  Boid boid_j({1773.37f, 247.55f}, {-2.03f, 6.59f});
  Boid boid_k({1281.38f, 763.62f}, {-6.02f, -8.50f});
  Boid boid_l({997.70f, 451.10f}, {-9.92f, -1.23f});
  Boid boid_m({950.03f, 609.44f}, {-8.56f, -8.77f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_k);
  flock.push_back(boid_l);
  flock.push_back(boid_m);
  stats = calculateStatistics(flock, start_time);
  CHECK(stats.sigma_d == doctest::Approx(309.308f).epsilon(0.001f));
}
TEST_CASE("Testing v_mean with five boids") {
  std::vector<Boid> flock;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  Stats stats{};
  Boid boid_i({530.99f, 797.67f}, {-2.14f, 6.31f});
  Boid boid_j({720.72f, 584.50f}, {-1.76f, 0.05f});
  Boid boid_k({460.19f, 332.93f}, {9.48f, -2.90f});
  Boid boid_l({661.79f, 701.81f}, {-1.17f, 7.93f});
  Boid boid_m({976.48f, 251.68f}, {4.39f, 6.82f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_k);
  flock.push_back(boid_l);
  flock.push_back(boid_m);
  stats = calculateStatistics(flock, start_time);
  CHECK(stats.v_mean == doctest::Approx(4.045f).epsilon(0.001f));
}

TEST_CASE("Testing sigma_v with five boids") {
  std::vector<Boid> flock;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  Stats stats{};
  Boid boid_i({530.99f, 797.67f}, {-2.14f, 6.31f});
  Boid boid_j({720.72f, 584.50f}, {-1.76f, 0.05f});
  Boid boid_k({460.19f, 332.93f}, {9.48f, -2.90f});
  Boid boid_l({661.79f, 701.81f}, {-1.17f, 7.93f});
  Boid boid_m({976.48f, 251.68f}, {4.39f, 6.82f});
  flock.push_back(boid_i);
  flock.push_back(boid_j);
  flock.push_back(boid_k);
  flock.push_back(boid_l);
  flock.push_back(boid_m);
  stats = calculateStatistics(flock, start_time);
  CHECK(stats.sigma_v == doctest::Approx(6.964f).epsilon(0.001f));
}