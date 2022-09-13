#include "catch2/catch2.hpp"
#include "liblinsys/mimo_struct.h"

const double tol = 1e-2;


TEST_CASE("Mimo_struct_suite", "[mimoStruct]") {

    SECTION("testMultiply") {
        int n = 3;
        Matrix A_identity = create_matrix(n, n);
        Matrix A_zero = create_matrix(n, n);
        Matrix A_one = create_matrix(n, n);
        Vector b = create_vector(n);
        for (int i = 0; i < n; i++) {
            b.data[i] = i;
            for (int j = 0; j < n; j++) {
                A_zero.data[j + i * n] = 0.0;
                A_one.data[j + i * n] = 1.0;
                if (i == j)
                    A_identity.data[j + n * i] = 1.0;
                else
                    A_identity.data[j + n * i] = 0.0;
            }
        }

        Vector bb = multiply(A_identity, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(b.data[i]).epsilon(tol));
        delete[] bb.data;

        bb = multiply(A_zero, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(0.0).epsilon(tol));
        delete[] bb.data;

        bb = multiply(A_one, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(3.0).epsilon(tol));
        delete[] bb.data;

        delete[] A_identity.data;
        delete[] A_zero.data;
        delete[] A_one.data;
        delete[] b.data;
    }

    SECTION("testSum") {
        int n = 3;
        Vector a_zero = create_vector(n);
        Vector a_one = create_vector(n);
        Vector b = create_vector(n);
        for (int i = 0; i < n; i++) {
            b.data[i] = i;
            a_zero.data[i] = 0.0;
            a_one.data[i] = 1.0;
        }

        Vector bb = sum(a_zero, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(b.data[i]).epsilon(tol));
        delete[] bb.data;

        bb = sum(a_one, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(b.data[i] + 1.0).epsilon(tol));
        delete[] bb.data;

        bb = sum(b, b);
        for (int i = 0; i < n; i++)
            REQUIRE(bb.data[i] == Approx(b.data[i] * 2.0).epsilon(tol));
        delete[] bb.data;

        delete[] a_zero.data;
        delete[] a_one.data;
        delete[] b.data;
    }

    SECTION("testPiControl") {
        int n = 3;
        double dt = 0.01;
        Vector integral_x = create_vector(n);
        Vector x = create_vector(n);
        Matrix Kp = create_matrix(n, n);
        Matrix Ki = create_matrix(n, n);
        for (int i = 0; i < n; i++) {
            x.data[i] = 3.0;
            integral_x.data[i] = 0.0;
            for (int j = 0; j < n; j++) {
                Kp.data[j + i * n] = 0.0;
                Ki.data[j + i * n] = 0.0;
            }
        }

//          pi_control(x, integral_x, dt=0.1, kp=0, ki=0)
        Vector u = pi_control(x, integral_x, Kp, Ki, dt);
        for (int i = 0; i < n; i++)
            REQUIRE(u.data[i] == Approx(0.0).epsilon(tol));
        for (int i = 0; i < n; i++)
            REQUIRE(integral_x.data[i] == Approx(0.03).epsilon(tol));
        delete[] u.data;

        for (int i = 0; i < n; i++)
            Kp.data[i + i * n] = 2.0;
        u = pi_control(x, integral_x, Kp, Ki, dt);
        for (int i = 0; i < n; i++)
            REQUIRE(u.data[i] == Approx(-6.0).epsilon(tol));
        for (int i = 0; i < n; i++)
            REQUIRE(integral_x.data[i] == Approx(0.06).epsilon(tol));
        delete[] u.data;

        for (int i = 0; i < n; i++) {
            Kp.data[i + i * n] = 0.0;
            Ki.data[i + i * n] = 2.0;
        }
        u = pi_control(x, integral_x, Kp, Ki, dt);
        for (int i = 0; i < n; i++)
            REQUIRE(u.data[i] == Approx(-0.18).epsilon(tol));
        for (int i = 0; i < n; i++)
            REQUIRE(integral_x.data[i] == Approx(0.09).epsilon(tol));
        delete[] u.data;

        delete[] x.data;
        delete[] integral_x.data;
        delete[] Kp.data;
        delete[] Ki.data;
    }

    SECTION("testSimulate") {
//        simulate(A, B, x, u, nx, nu, dt)
        int nx = 2;
        int nu = 2;
        double dt = 0.1;
        Matrix A = create_matrix(nx, nx);
        Matrix B = create_matrix(nx, nu);
        Vector x = create_vector(nx);
        Vector u = create_vector(nu);
        x.data[0] = 3.0;
        x.data[1] = 2.0;
        u.data[0] = 3.97863;
        u.data[1] = 7.12346;
        A.data[0] = 0.0;
        A.data[1] = 2.5;
        A.data[2] = 0.5;
        A.data[3] = 1.0;
        B.data[0] = 0.0;
        B.data[1] = 0.0;
        B.data[2] = 0.0;
        B.data[3] = 0.0;

//        simulate(a, b, x, u, dt=0.1)
        Vector x_next = simulate(A, B, x, u, dt);
        REQUIRE(x_next.data[0] == Approx(3.5).epsilon(tol));
        REQUIRE(x_next.data[1] == Approx(2.35).epsilon(tol));
        delete[] x_next.data;

        x.data[0] = 3.0;
        x.data[1] = 2.0;
        u.data[0] = 3.8;
        u.data[1] = 7.1;
        A.data[0] = 0.0;
        A.data[1] = 0.0;
        A.data[2] = 0.0;
        A.data[3] = 0.0;
        B.data[0] = 0.0;
        B.data[1] = 1.0;
        B.data[2] = 2.0;
        B.data[3] = 0.0;
        x_next = simulate(A, B, x, u, dt);
        REQUIRE(x_next.data[0] == Approx(3.71).epsilon(tol));
        REQUIRE(x_next.data[1] == Approx(2.76).epsilon(tol));
        delete[] x_next.data;

        delete[] A.data;
        delete[] B.data;
        delete[] x.data;
        delete[] u.data;
    }

    SECTION("testSimulateClosedLoop") {
// simulate_closed_loop(a, b, x0, dt, kp, ki, x_traj, N)
        int nx = 2;
        int nu = 2;
        int N = 5;
        double dt = 1.0;
        Matrix A = create_matrix(nx, nx);
        Matrix B = create_matrix(nx, nu);
        Vector x0 = create_vector(nx);
        Vector x_final = create_vector(nx);
        x0.data[0] = 3.0;
        x0.data[1] = 2.0;
        A.data[0] = 0.0;
        A.data[1] = 2.0;
        A.data[2] = 0.0;
        A.data[3] = 1.0;
        B.data[0] = 0.0;
        B.data[1] = 2.0;
        B.data[2] = 0.0;
        B.data[3] = 8.0;
        Matrix Kp = create_matrix(nu, nx);
        Matrix Ki = create_matrix(nu, nx);
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                Kp.data[j + i * nx] = 0.0;
                Ki.data[j + i * nx] = 0.0;
            }
        }
        {
            simulate_closed_loop(A, B, x0, dt, Kp, Ki, x_final, N);
            REQUIRE(x_final.data[0] == Approx(127.0).epsilon(tol));
            REQUIRE(x_final.data[1] == Approx(64.0).epsilon(tol));
        }
        {
            for (int i = 0; i < nx; i++) {
                Kp.data[i + i * nx] = 1.0;
                Ki.data[i + i * nx] = 0.0;
            }
            simulate_closed_loop(A, B, x0, dt, Kp, Ki, x_final, N);
            REQUIRE(x_final.data[0] == Approx(3.0).epsilon(tol));
            REQUIRE(x_final.data[1] == Approx(-15552.0).epsilon(tol));
        }
        delete[] A.data;
        delete[] B.data;
        delete[] Kp.data;
        delete[] Ki.data;
        delete[] x0.data;
        delete[] x_final.data;
    }
}


