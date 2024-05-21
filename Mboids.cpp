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
#include <iostream>
#include <cmath>
#include <numeric>  //per std::accumulate
#include <vector>

void InParams () {

    Sim sim{};
    double s1{};
    double a1{};
    double c1{};
    double d1{};
    double ds1{};

    std::cin >> s1 >> a1 >> c1 >> d1 >> ds1;
    sim.GetParams (s1, a1, c1, d1, ds1 ) ;
}

void Sim::GetParams(double s, double a, double c, double d , double ds) {

s_ = s ;
a_ = a ;
c_ = c ;
d_ = d ;
d_s_ = ds ;
}

// funzione per aggiungere boid, utile per test:
void Sim::add(Boid B) { stormo_.push_back(B); }  // as easy as that

// norma di una velocità di un boid mi torna utile per le formule:
double norm_v(Boid const& B) {
  return std::sqrt(std::pow(B.v_x, 2) + std::pow(B.v_y, 2));
}

// distanza tra due boid, utilità da definire :
double abs_distance(Boid const& A, Boid const& B) {
  return std::sqrt(std::pow((A.x - B.x), 2) + std::pow((A.y - B.y), 2));
}

// funzione statistica:
Stats Sim::statistics() {
  double N = stormo_.size();  // poi sarà il parametro N

  double sum_v_x{};
  double sum_v_y{};
  for (Boid B : stormo_) {
    sum_v_x += B.v_x;
    sum_v_y += B.v_y;
  }  // somme delle velocità coordinata per coordinata
  double v_media = std::sqrt(pow(sum_v_x, 2) + std::pow(sum_v_y, 2)) /
                   N;  // velocità media in modulo
  double sigma_v =
      std::sqrt(std::accumulate(stormo_.begin(), stormo_.end(), 0.,
                                [&v_media](double sum, Boid const& B) {
                                  return sum +=
                                         std::pow((norm_v(B) - v_media), 2);
                                })) /
      std::sqrt(N);  // deviazione standard della velocità, DA TESTARE

  double d_media{};
  std::vector<double>
      distances;  // lo faccio con un vettore così lo posso usare sia nella
                  // distanza media che nella sigma
  for (size_t i{}; i != stormo_.size(); ++i) {
    for (size_t j = i + 1; j != stormo_.size(); ++j) {
      distances.push_back(abs_distance(stormo_[i], stormo_[j]));
    }
  }
  d_media =
      std::accumulate(distances.begin(), distances.end(), 0.) /
      distances.size();  // distanza media. L'ho
                         // calcolata con un doppio for loop che praticamente
                         // fa tutte le combinazioni di distanze e poi divide
                         // per il numero di combinazioni
  double sigma_d{};
  sigma_d = std::sqrt(std::accumulate(distances.begin(), distances.end(), 0.,
                                      [&d_media](double d, double sum) {
                                        return sum += std::pow(d - d_media, 2);
                                      }) /
                      distances.size());  // deviazione standard, DA TESTARE

  return {v_media, d_media, sigma_v,
          sigma_d};  // importantissimo l'ordine, sennò la struct va a fanculo
} 
int main(){}