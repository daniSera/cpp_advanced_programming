///Prima Parte

/// Commento
// In tutto il progetto vengono usati come parametri delle funzioni (dove possibile) dei valori const in quanto
// il cambio implicito tra non const e const è ammissibile mentre il cambio implicito inverso è negato.

double simulate (const double &a, const double &b, const double &x, const double &u, const double &dt) {
    ///Controlli
    // la grandezza dt deve essere positiva

    double contrdt = (dt > 0) ? dt : -dt;

    ///Fine Controlli

    return x + (contrdt * ((a * x) + (b * u)));
}

double pi_control (const double &x, double &integral_x, const double &kp, const double &ki, const double &dt) {
    ///Controlli
    // la grandezza dt deve essere positiva

    double contrdt = (dt > 0) ? dt : -dt;

    ///Fine Controlli

    integral_x += x * contrdt;
    return ((-kp) * x) - (ki * integral_x);
}

void filter (const double *a, double *b, const int &n, const int &size) {

    ///Controlli
    // la grandezza n, size deve essere positiva

    int contrn = (n > 0) ? n : -n;
    int contrsize = (size > 0) ? size : -size;

    ///Fine Controlli

    int c;
    for (int i = 0; i < contrn; i++) {
        b[i] = a[i];
        c = 1;
        for (int j = 1; j < contrsize - (contrsize % 2); j++) {
            /// Commento
            //  A prescindere che size sia positivo o negativo, in questo loop lo considero POSITIVO
            // in quanto il termine centrale viene gia considerato prima.

            if (((i - j) >= 0) && ((i + j) < n)) {
                b[i] += a[i - j] + a[i + j];
                c += 2;
            } else if ((i - j) < 0) {
                b[i] += a[i + j];
                c++;
            } else {
                b[i] += a[i - j];
                c++;
            }
        }
        b[i] /= c;
    }
}

void simulate_closed_loop (const double &a, const double &b, const double &x0, const double &dt, const double &kp,
                           const double &ki, double *x_traj, const int &N) {

    ///Controlli
    // la grandezza N deve essere positiva

    int contrN = (N > 0) ? N : -N;

    ///Fine Controlli

    double integral_x = 0;
    x_traj[0] = x0;
    for (int i = 1; i <= contrN; i++) // il for va da 1 fino N compreso per fare tutti i step richiesti
        x_traj[i] = simulate(a, b, x_traj[i - 1], pi_control(x_traj[i - 1], integral_x, kp, ki, dt), dt);

    /// Commento
    // come discusso sul canale Telegram non viene inserita la funzione FILTER per filtrare i rsultati.
    // E' possibile vedere la versione con FILTER negli upload precedenti

}