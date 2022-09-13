/// Seconda Parte

#include <iostream>

using namespace std;

void print_array (const double *a, const int &n) {
    for (int i = 0; i < n; i++)
        cout << a[i] << " ";
    cout << endl;
}

double *multiply (const double *A, const double *b, const int &r, const int &c) {

    ///Controlli
    // le grandezze r e c devono essere positive

    int contrR = (r > 0) ? r : -r;
    int contrC = (c > 0) ? c : -c;

    ///Fine Controlli

    double *mult = new double[contrR];
    for (int i = 0; i < contrR; i++) {
        mult[i] = 0;
        for (int j = 0; j < contrC; j++) {
            mult[i] += A[(i * contrC) + j] * b[j];

            /// Commento
            //  tramite i * c viene considerato l'ordine ROW-MAJOR.
        }
    }
    return mult;
}

double *sum (const double *a, const double *b, const int &n) {

    ///Controlli
    // la grandezza n deve essere positiva

    int contr = (n > 0) ? n : -n;

    ///Fine Controlli

    double *s = new double[contr];
    for (int i = 0; i < contr; i++)
        s[i] = a[i] + b[i];
    return s;
}

double *pi_control (const double *x, double *integral_x, const int &nx, const double *Kp, const double *Ki,
                    const int &nu, const double &dt) {
    ///Controlli
    // la grandezza nx, nu, dt deve essere positiva

    int contrnx = (nx > 0) ? nx : -nx;
    int contrnu = (nu > 0) ? nu : -nu;
    double contrdt = (dt > 0) ? dt : -dt;

    ///Fine Controlli

    double *u = new double[contrnu];
    double *kpx;
    double *kiint;
    for (int j = 0; j < contrnx; j++)
        integral_x[j] += x[j] * contrdt;
    kpx = multiply(Kp, x, nu, nx); // fa da sè i controlli
    kiint = multiply(Ki, integral_x, nu, nx); // fa da sè i controlli

    /// Commento
    //  I pointer kpx e kiint vengono utilizzati per gestire l'allocazione dinamica di memoria,
    //  a fine funzione verranno eliminati i vettori generati dalla funzione MULTIPLY.

    for (int i = 0; i < contrnu; i++)
        u[i] = -kpx[i] - kiint[i];
    delete[] kpx;
    delete[] kiint;
    return u;
}

double *simulate (const double *A, const double *B, const double *x, const double *u, const int &nx,
                  const int &nu, const double &dt) {

    ///Controlli
    // la grandezza nx, dt deve essere positiva

    int contrnx = (nx > 0) ? nx : -nx;
    double contrdt = (dt > 0) ? dt : -dt;

    ///Fine Controlli

    double *var = new double[contrnx];
    double *A_x = 0;
    double *B_u = 0;
    A_x = multiply(A, x, nx, nx); // fa da sè i controlli
    B_u = multiply(B, u, nx, nu); // fa da sè i controlli

    /// Commento
    //  I pointer A_x e B_u vengono utilizzati per gestire l'allocazione dinamica di memoria,
    //  a fine funzione verranno eliminati i vettori generati dalla funzione MULTIPLY.

    for (int i = 0; i < contrnx; i++)
        var[i] = ((A_x[i] + B_u[i]) * contrdt) + x[i];
    delete[] A_x;
    delete[] B_u;
    return var;
}

void simulate_closed_loop (const double *A, const double *B, const double *x0, const int &nx, const int &nu,
                           const double &dt, const double *Kp, const double *Ki, double *x_final, const int &N) {

    ///Controlli
    // la grandezza nx, N deve essere positiva

    int contrnx = (nx > 0) ? nx : -nx;
    int contrN = (N > 0) ? N : -N;

    ///Fine Controlli

    double *integral_x = new double[contrnx];
    double *u = 0;
    double *pt = 0;
    for (int i = 0; i < contrnx; i++) {
        integral_x[i] = 0;
        x_final[i] = x0[i];
    }
    for (int i = 0; i < contrN; i++) {
        u = pi_control(x_final, integral_x, nx, Kp, Ki, nu, dt); // fa da sè i controlli

        /// Commento
        //  Il pointer u viene utilizzato per gestire l'allocazione dinamica di memoria,
        //  a fine iterazione verrà eliminato il vettore generato dalla funzione PI_CONTROL.

        pt = simulate(A, B, x_final, u, nx, nu, dt); // fa da sè i controlli

        /// Commento
        //  Il pointer pt viene utilizzato per gestire l'allocazione dinamica di memoria,
        //  a fine iterazione verrà eliminato il vettore generato dalla funzione SIMULATE.

        for (int j = 0; j < contrnx; j++)
            x_final[j] = pt[j];
        delete[] u;
        delete[] pt;
    }
    delete[] integral_x;
}

