#ifndef ES01_MIMO_STRUCT_H
#define ES01_MIMO_STRUCT_H

/**
 * Struttura dati per rappresentare una matrice
 */
struct Matrix {
    double *data;   // puntatore ai dati
    int nr;         // numero di righe
    int nc;         // numero di colonne
};

/**
 * Struttura dati per rappresentare un vettore
 */
struct Vector {
    double *data;   // puntatore ai dati
    int n;          // numero di elementi
};

/**
 * Crea un vettore
 * @param n Dimensione del vettore
 * @return Vettore di dimensione n
 */

Vector create_vector (const int &n);

/**
 * Crea una matrice
 * @param nr Numero di righe della matrice
 * @param nc Numero di colonne della matrice
 * @return Matrice con nr righe e nc colonne
 */

Matrix create_matrix (const int &nr, const int &nc);

/**
 * Moltiplicazione tra la matrice A ed il vettore b
 * @param A Matrice
 * @param b Vettore
 * @return Il risultato del prodotto A*b
 */

Vector multiply (const Matrix &A, const Vector &b);

/**
 * Somma tra vettori
 * @param a Primo vettore
 * @param b Secondo vettore
 * @return Il risultato della somma a+b
 */

Vector sum (const Vector &a, const Vector &b);

/**
 * Controllo proporzionale-integrale: u = -kp*x - ki*integral(x)
 * dove integral(x) è l'integrale di x.
 * @param x Stato corrente
 * @param integral_x Valore corrente dell'integrale di x
 * @param Kp Guadagno proporzionale (matrice nu-nx)
 * @param Ki Guadagno integrale (matrice nu-nx)
 * @param dt Durata del time step
 * @return Valore del controllo u
 */

Vector
pi_control (const Vector &x, const Vector &integral_x, const Matrix &Kp, const Matrix &Ki, const double &dt = 0.1);

/**
 * Simulazione di un time step per il sistema dinamico lineare
 *   dx = A*x + B*u
 * dove dx è la derivata di x rispetto al tempo.
 * La simulazione è basata sul metodo di integrazione numerica "Eulero esplicito", ovvero
 *   x := x + dt * dx
 * @param A Matrice di stato
 * @param B Matrice di controllo
 * @param x Vettore di stato corrente
 * @param u Vettore di controllo corrente
 * @param dt Durata del time step
 * @return Valore dello stato x alla fine del time step
 */

Vector simulate (const Matrix &A, const Matrix &B, const Vector &x, const Vector &u, const double &dt = 0.1);

/**
 * Funzione per simulare il sistema in ciclo chiuso.
 * @param A Matrice di stato
 * @param B Matrice di controllo
 * @param x0 Stato iniziale
 * @param dt Time step usato per la simulazione
 * @param Kp Guadagno proporzionale del controllore (matrice nx-nu)
 * @param Ki Guadagno integrale del controllore (matrice nx-nu)
 * @param x_final Valore finale dello stato (output)
 * @param N Numero di time step da simulare
 */

void simulate_closed_loop (const Matrix &A, const Matrix &B, const Vector &x0, const double &dt, const Matrix &Kp,
                           const Matrix &Ki, const Vector &x_final, const int &N);

#endif //ES01_MIMO_STRUCT_H
