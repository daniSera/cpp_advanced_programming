/// Terza Parte

#include "liblinsys/mimo_struct.h"
#include <iostream>
#include "liblinsys/mimo.h"

using namespace std;

/// Commento
// Nelle successive funzioni applico l'ideale che le fuzioni vengano eseguite comunuqe anche se
// le dimensioni di array e matrici sono differenti, tutelandomi e prendendo la dimensione più piccola
// tra quelle disponibili (per non eccedere fuori dai vettori).

Vector create_vector (const int &n) {
    ///Controlli

    int contr = (n > 0) ? n : -n;

    ///Fine Controlli
    Vector str{new double[contr], contr};
    for (int i = 0; i < contr; i++)
        str.data[i] = 0; // per essere sicuro dei valori inizializzati
    return str;

    /// Commento
    // in CREATE_VECTOR viene effettuata una allocazione dinamica, restituendo
    // il vettore ph ci si aspetta che il chiamante effettui un delete[].

}

Matrix create_matrix (const int &nr, const int &nc) {
    ///Controlli

    int contr = ((nr * nc) > 0) ? (nr * nc) : -(nr * nc);

    ///Fine Controlli
    Matrix str{new double[contr], (nr > 0) ? nr : -nr, (nc > 0) ? nc : -nc};
    for (int i = 0; i < contr; i++)
        str.data[i] = 0; // per essere sicuro dei valori inizializzati
    return str;

    /// Commento
    // in CREATE_MATRIX viene effettuata una allocazione dinamica, restituendo
    // il vettore ph ci si aspetta che il chiamante effettui un delete[].

}

Vector multiply (const Matrix &A, const Vector &b) {

    /// Controlli
    // Il numero di colonne deve essere = alla lunghezza di b per evitare di sovrascrivere
    // altre informazioni o considerare info sbagliate,
    // prendo la dimensioni più piccola tra le due, in modo che comunque la funzione venga eseguita

    int contr = (A.nc <= b.n) ? A.nc : b.n;

    /// Fine Controlli

    Vector ph = create_vector(contr);
    double *pt = multiply(A.data, b.data, A.nr, contr);
    for (int i = 0; i < A.nr; i++)
        ph.data[i] = pt[i];
    delete[] pt;
    return ph;

    // guardare se è meglio fare la versione di mimo invece che richiamarla, xo lo scopo delle funzioni
    // è quello di riutilizzare codice

}

Vector sum (const Vector &a, const Vector &b) {

    /// Controlli
    // Le dimensioni di a e b devono essere le medesime, si prende come misura
    // la minore delle 2 per non eccedere dal vettore più piccolo,
    // in modo che comunque la funzione venga eseguita

    int contr = (a.n <= b.n) ? a.n : b.n;

    /// Fine Controlli

    Vector ph = create_vector(contr);
    for (int i = 0; i < contr; i++)
        ph.data[i] = a.data[i] + b.data[i];
    return ph;
}

Vector pi_control (const Vector &x, const Vector &integral_x, const Matrix &Kp, const Matrix &Ki, const double &dt) {

    /// Controlli
    // Nelle moltiplicazioni le grandezze devono combaciare. Vengono prese le
    // dimensioni minori per non eccedere dai vettori.

    /// Righe (lunghezza u)
    int contrR = (Kp.nr <= Ki.nr) ? Kp.nr : Ki.nr;

    /// Colonne (lunghezza x , integral_x)
    int contrC = (min(Kp.nc, x.n) <= min(Ki.nc, integral_x.n)) ? min(Kp.nc, x.n) : min(Ki.nc, integral_x.n);

    /// Fine Controlli

    Vector u = create_vector(contrR);
    double *pt = pi_control(x.data, integral_x.data, contrC, Kp.data, Ki.data, contrR, dt);
    for (int i = 0; i < contrR; i++)
        u.data[i] = pt[i];
    delete[] pt;
    return u;
}

Vector simulate (const Matrix &A, const Matrix &B, const Vector &x, const Vector &u, const double &dt) {
    /// Controlli
    // Nelle moltiplicazioni le grandezze devono combaciare. Vengono prese le
    // dimensioni minori per non eccedere dai vettori.

    int contrR = (A.nr <= B.nr) ? A.nr : B.nr; /// Righe

    int contrX = (A.nc <= x.n) ? A.nc : x.n; /// Colonne

    int contrU = (B.nc <= u.n) ? B.nc : u.n; /// Colonne

    // sarebbe da valutare anche il minimo tra i due valori di colonne

    /// Fine Controlli

    Vector ph = create_vector(contrR);
    double *pt = simulate(A.data, B.data, x.data, u.data, contrX, contrU, dt);
    for (int i = 0; i < contrR; i++)
        ph.data[i] = pt[i];
    delete[] pt;
    return ph;
}

void simulate_closed_loop (const Matrix &A, const Matrix &B, const Vector &x0, const double &dt, const Matrix &Kp,
                           const Matrix &Ki, const Vector &x_final, const int &N) {

    /// Commenti
    // la versione che richiede meno codice è di richiamare la funzione
    // sviluppata in mimo.cpp

    /// Controlli
    // Nelle moltiplicazioni le grandezze devono combaciare. Vengono prese le
    // dimensioni minori per non eccedere dai vettori.

    int contrR = min(min(A.nr, B.nr), min(x_final.n, x0.n)); /// Righe

    int contrU = min(min(B.nc, Kp.nr), Ki.nr); /// Controllo vettore u

    /// Fine Controlli

    simulate_closed_loop(A.data, B.data, x0.data, contrR, contrU, dt, Kp.data, Ki.data, x_final.data, N);
}










