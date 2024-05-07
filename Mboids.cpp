/* #include vari

Boid struct
Struct stats

Classe Stormo{
vector<Boid> (matrice)
params

ciclo per ogni elemento del vettore Stormo
    funzioni correzione velocità
        aggiornamento

funzione statistiche
}

#main

get params (s, a, c, d, d_s, n, T=60s, d_t=1s)
Generazione casuale in un altro file
push_back dei boid per ogni 1->n

ciclo per il tempo 1->60
    ciclo per ogni elemento della matrice Stormo 1->n

vettore<stats> che contiene le statistiche ogni tot tempo
cout tutto
*/

#include "Mboids.hpp"  //header file

#include <cmath>
#include <numeric>  //per std::accumulate
#include <vector>

// funzione per aggiungere boid, utile per test:
void Sim::add(Boid B) { stormo_.push_back(B); }  // as easy as that

// norma di una velocità di un boid mi torna utile per le formule:
double norm_v(Boid const& B) {
  return std::sqrt(std::pow(B.v_x, 2) + std::pow(B.v_y, 2));
}

// funzione statistica:
Stats Sim::statistics() {
  double N = stormo_.size();  // poi sarà il parametro N
  double sum_v_x = std::accumulate(
      stormo_.begin(), stormo_.end(), 0.,
      [](double sum, Boid const& B) { return sum += B.v_x; });  // somma v_x
  double sum_v_y = std::accumulate(
      stormo_.begin(), stormo_.end(), 0.,
      [](double sum, Boid const& B) { return sum += B.v_y; });  // somma v_y
  double v_med = std::sqrt(pow(sum_v_x, 2) + std::pow(sum_v_y, 2)) /
                 N;  // velocità media in modulo

  /*per la velocità media non in modulo:
      double v_x_med = sum_v_x / N;
      double v_y_med = sum_v_y /N;
  */
  double sigma_v =
      std::sqrt(std::accumulate(stormo_.begin(), stormo_.end(), 0.,
                                [&v_med](double sum, Boid const& B) {
                                  return sum +=
                                         std::pow((norm_v(B) - v_med), 2);
                                })) /
      std::sqrt(N);  // deviazione standard della velocità, DA TESTARE
  double d_med{};    // distanza media che qualcuno mi dovrà spiegare
  double sigma_d{};  // uguale

  return {v_med, d_med, sigma_v,
          sigma_d};  // importantissimo l'ordine, sennò la struct va a fanculo
}
