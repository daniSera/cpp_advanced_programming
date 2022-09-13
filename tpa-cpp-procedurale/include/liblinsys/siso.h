#ifndef ES01_SISO_H
#define ES01_SISO_H

/**
 * Controllo proporzionale-integrale: u = -kp*x - ki*integral(x)
 * dove integral(x) è l'integrale di x.
 * @param x Stato corrente x[n]
 * @param integral_x Valore corrente dell'integrale di x
 * @param kp Guadagno proporzionale
 * @param ki Guadagno integrale
 * @param dt Durata del time step
 * @return Valore del controllo u
 */
double pi_control (const double &x, double &integral_x, const double &kp = 0, const double &ki = 0,
                   const double &dt = 0.1);

/**
 * Simulazione di un time step per il sistema dinamico lineare
 *   dx = a*x + b*u
 * dove dx è la derivata di x rispetto al tempo.
 * La simulazione è basata sul metodo di integrazione numerica "Eulero esplicito", ovvero
 *   x := x + dt * dx
 * @param a Coefficiente di x
 * @param b Coefficiente di u
 * @param x Stato corrente
 * @param u Controllo corrente
 * @param dt Durata del time step
 * @return Valore dello stato x alla fine del time step
 */
double simulate (const double &a, const double &b, const double &x, const double &u, const double &dt = 0.1);

/**
 * Filtra la traiettoria specificata usando un finestra glissante della dimensione specificata.
 * Il filtraggio consiste nel sostituire l'elemento i-esimo della traiettoria con la media
 * degli elementi attorno all'i-esimo. Se window_size==3 allora l'elemento i-esimo sarà dato
 * dalla media degli elementi in posizione i-1, i ed i+1.
 * @param a Lista di elementi in ingresso, ovvero traiettoria da filtrare
 * @param b Lista di elementi in uscita, ovvero traiettoria filtrata
 * @param n Numero di elementi in a
 * @param window_size Dimensione della finestra glissante
 */
void filter (const double *a, double *b, const int &n, const int &size = 3);

/**
 * Funzione per simulare il sistema ad anello chiuso.
 * @param a Coefficiente di x
 * @param b Coefficiente di u
 * @param x0 Stato iniziale
 * @param dt Time step usato per la simulazione
 * @param kp Guadagno proporzionale del controllore
 * @param ki Guadagno integrale del controllore
 * @param x_traj Traiettoria dello stato (output)
 * @param N Numero di time step da simulare
 */
void simulate_closed_loop (const double &a, const double &b, const double &x0, const double &dt, const double &kp,
                           const double &ki, double *x_traj, const int &N);

#endif //ES01_SISO_H
