#include "catch2/catch2.hpp"
#include "liblinsys/siso.h"

const double tol = 1e-2;


TEST_CASE("Siso_suite", "[siso]") {

    SECTION("testPiControl")
//    SECTION(testPiControl)
    {
        double integral_x = 0.0;
        double dt = 0.01;
        double x = 3.0;
        REQUIRE(pi_control(x, integral_x, 0.0, 0.0, dt) == Approx(0.0).epsilon(tol));
        REQUIRE(integral_x == Approx(0.03).epsilon(tol));
        REQUIRE(pi_control(x, integral_x, 2.0, 0.0, dt) == Approx(-6.0).epsilon(tol));
        REQUIRE(integral_x == Approx(0.06).epsilon(tol));
        REQUIRE(pi_control(x, integral_x, 0.0, 2.0, dt) == Approx(-0.18).epsilon(tol));
        REQUIRE(integral_x == Approx(0.09).epsilon(tol));
    }

    SECTION("testSimulate") {
//                   simulate(a, b, x, u, dt=0.1)
        REQUIRE(simulate(2.5, 0.0, 3.0, 3.873456) == Approx(3.75).epsilon(tol));
        REQUIRE(simulate(2.5, 1.2, 0.0, 3.0) == Approx(0.36).epsilon(tol));
        REQUIRE(simulate(2.5, 1.2, 6.2, 3.0, 0.0) == Approx(6.2).epsilon(tol));
    }



    SECTION("testFilter") {
// filter(a, b, n, window_size=3)
        const int n = 5;
        double a[n] = {1.0, 3.0, 5.0, 4.0, 0.0};
        double b[n];
        filter(a, b, n);
        REQUIRE(b[0] == Approx(2.0).epsilon(tol));
        REQUIRE(b[1] == Approx(3.0).epsilon(tol));
        REQUIRE(b[2] == Approx(4.0).epsilon(tol));
        REQUIRE(b[3] == Approx(3.0).epsilon(tol));
        REQUIRE(b[4] == Approx(2.0).epsilon(tol));

        double c[n];
        filter(a, c, n, 2);
        REQUIRE(c[0] == Approx(2.0).epsilon(tol));
        REQUIRE(c[1] == Approx(3.0).epsilon(tol));
        REQUIRE(c[2] == Approx(4.0).epsilon(tol));
        REQUIRE(c[3] == Approx(3.0).epsilon(tol));
        REQUIRE(c[4] == Approx(2.0).epsilon(tol));
    }


    SECTION("testSimulateClosedLoop") {
// simulate_closed_loop(a, b, x0, dt, kp, ki, x_traj, N)
        const int N = 5;
        double dt = 0.1;
        double x0 = 3.14;
        double kp = 0.0;
        double ki = 0.0;
        {
            double x_traj[N + 1];
            simulate_closed_loop(0.0, 0.0, x0, dt, kp, ki, x_traj, N);
            for (int i = 0; i < N; i++)
                REQUIRE(x_traj[i] == Approx(x0).epsilon(tol));
        }
        {
            double x_traj[N + 1];
            kp = 2.0;
            simulate_closed_loop(1.0, 0.5, x0, dt, kp, ki, x_traj, N);
            for (int i = 0; i < N; i++)
                REQUIRE(x_traj[i] == Approx(x0).epsilon(tol));
        }
    }
}


