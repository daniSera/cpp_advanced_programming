# tpa2020_assignment_01
Primo compito del corso "Tecniche di Programmazione Avanzata" del Dipartimento di Ingegneria Industriale, anno accademico 2019/20.

Questo compito si concentra sulla programmazione procedurale in C++.
In questa repository lo studente può già trovare i file `CMakeLists.txt` (da non modificare), i file `.cpp` vuoti da riempire (cartella `src`), i file header `.h` parzialmente implementati (cartella `include`) ed i file contenenti gli unit test (cartella `test`). 

## Uso dei file header
Nei file header sono già presenti delle dichiarazioni parziali delle funzioni da implementare.
Queste dichiarazioni riportano
* nome della funzione
* nome e descrizione dei parametri

È compito dello studente completare le dichiarazioni aggiungendo:
* tipo di ritorno 
* tipo dei parametri

## Uso dei test
Lo studente è incoraggiato a servirsi dei test per verificare che la sua implementazione delle funzioni sia corretta.
Se dei test falliscono allora significa che l'implementazione non è corretta.
Non è vero però il contrario: i test potrebbero passare tutti anche se l'implementazione contiene degli errori.
La responsabilità finale di giudicare se una funzione è implementata correttamente è dello studente, ed i test devono essere
visti solo come uno strumento di supporto a tale scopo.


# Descrizione del compito
Il compito si struttura in 3 parti, ognuna della quali è un'estensione della precedente, quindi si consiglia di completare una parte prima di cominciare a lavorare sulla seguente.
Il codice relativo alla prima parte andrà riportato nei file siso.h e siso.cpp.
Per la seconda parte useremo i file mimo.h e mimo.cpp.
Infine, per la terza ed ultima parte useremo i file mimo_struct.h e mimo_struct.cpp.

Per rappresentare numeri reali si devono utilizzare variabili di tipo `double`.

## Parte 1: Sistemi SISO
L'obiettivo di questa parte è sviluppare un piccolo sistema software di simulazione e controllo per sistemi lineari tempo-invarianti single-input single-output (SISO).

### Funzione `simulate`
Questa funzione simula il sistema dinamico siso per il tempo specificato `dt`.
Il sistema siso ha una dinamica del tipo:
```
dx = a * x + b * u
```
dove
* `x` è lo stato del sistema
* `u` è il controllo del sistema
* `dx` è la derivata di x rispetto al tempo
* `a` è il coefficiente dello stato
* `b` è il coefficiente del controllo

Dato che si tratta di sistemi SISO, tutti i valore nella lista sopra sono scalari (di tipo `double`).

Per simulare il sistema useremo un metodo di integrazione numerica molto semplice: [Eulero esplicito](https://it.wikipedia.org/wiki/Metodo_di_Eulero).
Dato lo stato corrente `x[n]` e la sua derivata `dx[n]`, il metodo di Eulero calcola il valore dello stato al prossimo istante discreto `n+1` come:
```
x[n+1] = x[n] + dt * dx[n]
```
dove `dt` è la durata del passo di integrazione. 
Se `dt` non è specificato si deve assumere il valore 0.1.

### Funzione `pi_control`
Il controllore da implementare è di tipo Proporzionale-Integrale (PI), ovvero:
```
u = - kp * x - ki * integral(x)
```
dove
* `kp` è il guadagno proporzionale (zero se non specificato)
* `ki` è il guadagno integrale (zero se non specificato)
* `integral(x)` è l'integrale di x

L'integrale di `x` al passo di integrazione `n` può essere approssimato con il metodo di Eulero:
```
x[0]*dt + x[1]*dt + ... + x[n-1]*dt + x[n]*dt
```
dove `dt` è la durata del passo di integrazione. 
Se `dt` non è specificato si deve assumere il valore 0.1.


### Funzione `filter`
Questa funzione filtra la traiettoria specificata usando una finestra mobile della dimensione specificata `window_size`. 
Il filtraggio consiste nel sostituire l'elemento i-esimo della traiettoria con la media
degli elementi attorno all'i-esimo. Se `window_size`==3 allora l'elemento i-esimo sarà dato
dalla media degli elementi in posizione i-1, i ed i+1.
Se `window_size` non dovesse essere un numero dispari, allora il metodo deve comportarsi come se fosse stato chiamato con un valore
uguale a `window_size+1`. 
Se `window_size` non è specificato si deve assumere il valore 3.


Per i primi e gli ultimi elementi della traiettoria, per i quali non sono disponibili tutti i valori nella finestra, l'algoritmo
deve limitarsi a calcolare la media tra i valori disponibili. Si fa riferimento ai test per maggiori chiarimenti sul comportamento 
atteso.

### Funzione `simulate_closed_loop`
Funzione per simulare il [sistema ad anello chiuso](https://it.wikipedia.org/wiki/Controllo_automatico#Controllo_ad_anello_chiuso_(retroazione)) usando le funzioni `pi_control` e `simulate`.

## Parte 2: Sistemi MIMO
In questa parte del compito lo studente deve implementare le funzioni `pi_control`, `simulate` e `simulate_closed_loop` per dei sistemi multi-input multi-output (MIMO).
Queste funzioni hanno le stesse funzionalità descritte sopra, ma questa volta:
* `x` e `u` sono vettori
* `A`, `B`, `Kp` e `Ki` sono matrici
In C++, vettori e matrici sono tipicamente rappresentati medianti array.
Le matrici devono essere salvate in memoria per riga (ordine [row-major](https://en.wikipedia.org/wiki/Row-_and_column-major_order)).

La dimensioni di questi vettori e matrici non sarà nota a tempo di compilazione, quindi non possono essere usati array statici.
Lo studente dovrà servirsi di array dinamici, allocando la memoria con l'operatore `new` e deallocandola non appena possibile
con l'operatore `delete`. La corretta gestione della memoria non è verificata esplicitamente dai test forniti, 
ma sarà presa in considerazione nella valutazione del compito.

Oltre alle tre funzioni menzionate sopra, lo studente dovrà anche implementare le funzioni `multiply` e `sum` per eseguenre le operazioni di 
moltiplicazione matrice-vettore e di somma tra vettori.
Queste funzioni saranno utili per l'implementazione di `pi_control` e `simulate`.

## Parte 3: Sistemi MIMO con Strutture
Allo scopo di ridurre il numero di parametri delle funzioni e di migliorarne l'usabilità, introduciamo le strutture `Vector` e `Matrix`.
Lo studente deve quindi implementare le nuove versioni delle funzioni `multiply`, `sum`, `pi_control`, `simulate` e `simulate_closed_loop`.
Per semplificare l'operazione ricorrente di creazione di `Vector` e `Matrix`, abbiamo anche aggiunto le funzioni `create_vector` e
`create_matrix`.
