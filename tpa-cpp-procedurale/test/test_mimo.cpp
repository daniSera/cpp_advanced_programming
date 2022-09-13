#include "catch2/catch2.hpp"
#include "liblinsys/mimo.h"

const double tol = 1e-2;


TEST_CASE("Mimo_suite", "[mimo]") {

    SECTION("testMultiply") {
        int n = 3;
        double *A_identity = new double[n * n];
        double *A_zero = new double[n * n];
        double *A_one = new double[n * n];
        double *b = new double[n];
        for (int i = 0; i < n; i++) {
            b[i] = i;
            for (int j = 0; j < n; j++) {
                A_zero[j + i * n] = 0.0;
                A_one[j + i * n] = 1.0;
                if (i == j)
                    A_identity[j + n * i] = 1.0;
                else
                    A_identity[j + n * i] = 0.0;
            }
        }

        double *bb = multiply(A_identity, b, n, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(b[i]).epsilon(tol));
        delete[] bb;

        bb = multiply(A_zero, b, n, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(0.0).epsilon(tol));
        delete[] bb;

        bb = multiply(A_one, b, n, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(3.0).epsilon(tol));
        delete[] bb;

        delete[] A_identity;
        delete[] A_zero;
        delete[] A_one;
        delete[] b;
    }


    SECTION("testSum") {
        int n = 3;
        double *a = new double[n];
        double *a_zero = new double[n];
        double *a_one = new double[n];
        double *b = new double[n];
        for (int i = 0; i < n; i++) {
            b[i] = i;
            a_zero[i] = 0.0;
            a_one[i] = 1.0;
        }

        double *bb = sum(a_zero, b, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(b[i]).epsilon(tol));
        delete[] bb;

        bb = sum(a_one, b, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(b[i] + 1.0).epsilon(tol));
        delete[] bb;

        bb = sum(b, b, n);
        for (int i = 0; i < n; i++)
            REQUIRE(bb[i] == Approx(2.0 * b[i]).epsilon(tol));
        delete[] bb;

        delete[] a_zero;
        delete[] a_one;
        delete[] b;
    }

    SECTION("testPiControl") {
        int nx = 3, nu = 3;
        double dt = 0.01;
        double *integral_x = new double[nx];
        double *x = new double[nx];
        double *Kp = new double[nu * nx];
        double *Ki = new double[nu * nx];
        for (int i = 0; i < nx; i++) {
            x[i] = 3.0;
            integral_x[i] = 0.0;
            for (int j = 0; j < nu; j++) {
                Kp[j + i * nx] = 0.0;
                Ki[j + i * nx] = 0.0;
            }
        }

//          pi_control(x, integral_x, n, dt=0.1, kp=0, ki=0)
        double *u = pi_control(x, integral_x, nx, Kp, Ki, nu, dt);
        for (int i = 0; i < nu; i++)
            REQUIRE(u[i] == Approx(0.0).epsilon(tol));
        for (int i = 0; i < nx; i++)
            REQUIRE(integral_x[i] == Approx(0.03).epsilon(tol));
        delete[] u;

        for (int j = 0; j < nu; j++) {
            Kp[j + j * nx] = 2.0;
        }
        u = pi_control(x, integral_x, nx, Kp, Ki, nu, dt);
        for (int i = 0; i < nu; i++)
            REQUIRE(u[i] == Approx(-6.0).epsilon(tol));
        for (int i = 0; i < nx; i++)
            REQUIRE(integral_x[i] == Approx(0.06).epsilon(tol));
        delete[] u;

        for (int j = 0; j < nu; j++) {
            Kp[j + j * nx] = 0.0;
            Ki[j + j * nx] = 2.0;
        }
        u = pi_control(x, integral_x, nx, Kp, Ki, nu, dt);
        for (int i = 0; i < nu; i++)
            REQUIRE(u[i] == Approx(-0.18).epsilon(tol));
        for (int i = 0; i < nx; i++)
            REQUIRE(integral_x[i] == Approx(0.09).epsilon(tol));
        delete[] u;

        delete[] x;
        delete[] integral_x;
        delete[] Kp;
        delete[] Ki;
    }


    SECTION("testSimulate") {
//        simulate(A, B, x, u, nx, nu, dt)
        int nx = 2;
        int nu = 2;
        double dt = 0.1;
        double *A = new double[nx * nx];
        double *B = new double[nx * nu];
        double *x = new double[nx];
        double *u = new double[nu];
        x[0] = 3.0;
        x[1] = 2.0;
        u[0] = 3.97863;
        u[1] = 7.12346;
        A[0] = 0.0;
        A[1] = 2.5;
        A[2] = 0.5;
        A[3] = 1.0;
        B[0] = 0.0;
        B[1] = 0.0;
        B[2] = 0.0;
        B[3] = 0.0;
//        simulate(a, b, x, u, dt=0.1)
        double *x_next = simulate(A, B, x, u, nx, nu, dt);
        REQUIRE(x_next[0] == Approx(3.5).epsilon(tol));
        REQUIRE(x_next[1] == Approx(2.35).epsilon(tol));
        delete[] x_next;

        x[0] = 3.0;
        x[1] = 2.0;
        u[0] = 3.8;
        u[1] = 7.1;
        A[0] = 0.0;
        A[1] = 0.0;
        A[2] = 0.0;
        A[3] = 0.0;
        B[0] = 0.0;
        B[1] = 1.0;
        B[2] = 2.0;
        B[3] = 0.0;
        x_next = simulate(A, B, x, u, nx, nu, dt);
        REQUIRE(x_next[0] == Approx(3.71).epsilon(tol));
        REQUIRE(x_next[1] == Approx(2.76).epsilon(tol));
        delete[] x_next;

        delete[] A;
        delete[] B;
        delete[] x;
        delete[] u;
    }

    SECTION("testSimulateClosedLoop") {
// simulate_closed_loop(a, b, x0, dt, kp, ki, x_traj, N)
        int nx = 2;
        int nu = 2;
        int N = 5;
        double dt = 1.0;
        double *A = new double[nx * nx];
        double *B = new double[nx * nu];
        double *x0 = new double[nx];
        double *x_final = new double[nx];
        x0[0] = 3.0;
        x0[1] = 2.0;
        A[0] = 0.0;
        A[1] = 2.0;
        A[2] = 0.0;
        A[3] = 1.0;
        B[0] = 0.0;
        B[1] = 2.0;
        B[2] = 0.0;
        B[3] = 8.0;
        double *Kp = new double[nu * nx];
        double *Ki = new double[nu * nx];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nu; j++) {
                Kp[j + i * nx] = 0.0;
                Ki[j + i * nx] = 0.0;
            }
        }
        {
            simulate_closed_loop(A, B, x0, nx, nu, dt, Kp, Ki, x_final, N);
            REQUIRE(x_final[0] == Approx(127.0).epsilon(tol));
            REQUIRE(x_final[1] == Approx(64.0).epsilon(tol));
        }
        {
            for (int j = 0; j < nu; j++) {
                Kp[j + j * nx] = 1.0;
            }
            simulate_closed_loop(A, B, x0, nx, nu, dt, Kp, Ki, x_final, N);
            REQUIRE(x_final[0] == Approx(3.0).epsilon(tol));
            REQUIRE(x_final[1] == Approx(-15552.0).epsilon(tol));
        }
        delete[] A;
        delete[] B;
        delete[] x0;
        delete[] x_final;
        delete[] Kp;
        delete[] Ki;
    }
}
