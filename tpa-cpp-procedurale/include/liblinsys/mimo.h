#ifndef ES01_MIMO_H
#define ES01_MIMO_H

/**
 * Stampa a video l'array specificato
 * @param a Puntatore all'array da stampare
 * @param n Dimensione dell'array
 */

void print_array (const double *a, const int &n);

/**
 * Moltiplicazione tra la matrice A ed il vettore b
 * @param A Matrice
 * @param b Vettore
 * @param r Numero di righe di A
 * @param c Numero di colonne di A
 * @return Il risultato del prodotto A*b
 */

double *multiply (const double *A, const double *b, const int &r, const int &c);

/**
 * Somma tra vettori
 * @param a Primo vettore
 * @param b Secondo vettore
 * @param n Dimensione di a e b
 * @return Il risultato della somma a+b
 */

double *sum (const double *a, const double *b, const int &n);

/**
 * Controllo proporzionale-egrale: u = -kp*x - ki*egral(x)
 * dove egral(x) è l'egrale di x.
 * @param x Stato corrente
 * @param egral_x Valore corrente dell'egrale di x
 * @param nx Dimensione di x
 * @param Kp Guadagno proporzionale (matrice nu-nx)
 * @param Ki Guadagno egrale (matrice nu-nx)
 * @param nu Dimensione di u
 * @param dt Durata del time step
 * @return Valore del controllo u
 */

double *pi_control (const double *x, double *integral_x, const int &nx, const double *Kp, const double *Ki,
                    const int &nu, const double &dt = 0.1);

/**
 * Simulazione di un time step per il sistema dinamico lineare
 *   dx = A*x + B*u
 * dove dx è la derivata di x rispetto al tempo.
 * La simulazione è basata sul metodo di egrazione numerica "Eulero esplicito", ovvero
 *   x := x + dt * dx
 * @param A Matrice di stato
 * @param B Matrice di controllo
 * @param x Vettore di stato corrente
 * @param u Vettore di controllo corrente
 * @param nx Dimensione di x
 * @param nu Dimensione di u
 * @param dt Durata del time step
 * @return Valore dello stato x alla fine del time step
 */

double *simulate (const double *A, const double *B, const double *x, const double *u, const int &nx, const int &nu,
                  const double &dt = 0.1);

/**
 * Funzione per simulare il sistema in ciclo chiuso.
 * @param A Matrice di stato
 * @param B Matrice di controllo
 * @param x0 Stato iniziale
 * @param nx Dimensione di x
 * @param nu Dimensione di u
 * @param dt Time step usato per la simulazione
 * @param Kp Guadagno proporzionale del controllore (matrice nx-nu)
 * @param Ki Guadagno egrale del controllore (matrice nx-nu)
 * @param x_final Valore finale dello stato (output)
 * @param N Numero di time step da simulare
 */

void simulate_closed_loop (const double *A, const double *B, const double *x0, const int &nx, const int &nu,
                           const double &dt, const double *Kp, const double *Ki, double *x_final, const int &N);

#endif //ES01_MIMO_H
