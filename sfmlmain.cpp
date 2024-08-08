#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/System.hpp>
#include <memory>

#include "Boids_ver_2.hpp"

const int width = sf::VideoMode::getDesktopMode().width;
const int height = sf::VideoMode::getDesktopMode().height;
const float max_speed = 8;

void draw(sf::RenderWindow& window, const Boid& boid) {
  sf::CircleShape shape(2);  // dimensioni del cerchio
  shape.setPosition(
      (boid.position.x),
      (boid.position.y));  // uso i float perché sfml funziona con i float
  shape.setFillColor(sf::Color::White);
  window.draw(shape);
}  // disegna il boid come un cerchio

/*void out_of_border(Boid& boid) {
  if (boid.position.x < 0) {
    boid.position.x = 0;
    boid.velocity.x = boid.velocity.x * -1;
  }
  if (boid.position.y < 0) {
    boid.position.y = 0;
    boid.velocity.y = boid.velocity.y * -1;
  }
  if (boid.position.x > width) {
    boid.position.x = width;
    boid.velocity.x = boid.velocity.x * -1;
  }
  if (boid.position.y > height) {
    boid.position.y = height;
    boid.velocity.y = boid.velocity.y * -1;
  }
}  // questa funzione dovrebbe in teoria essere responsabile dell'effetto
   // pacman. Vedendo i risultati questa scelta non so se è delle migliori
   // perché sminchia un po' le regole forse

void repulsive_border(Boid& boid) {
  if (boid.position.x < 80) {  // && boid.velocity.x <= 0) {
    boid.velocity.x +=
        (boid.velocity.norm()) * (2) *
        (1 / std::pow(boid.position.x,
                      0.5));  // il fattore additivo è per evitare
                              // che i boid si fermino in teoria
  }
  if (boid.position.y < 80) {  //&& boid.velocity.y <= 0) {
    boid.velocity.y +=
        (boid.velocity.norm()) * (2) * (1 / std::pow(boid.position.y, 0.5));
  }
  if (boid.position.x > (width - 80)) {  //&& boid.velocity.x >= 0) {
    boid.velocity.x += (boid.velocity.norm()) * (-2) *
                       (1 / std::pow((width - boid.position.x), 0.5));
  }
  if (boid.position.y > (height - 80)) {  //&& boid.velocity.y >= 0) {
    boid.velocity.y += (boid.velocity.norm()) * (-2) *
                       (1 / std::pow((height - boid.position.y), 0.5));
  }
}  // questa legge ha azione solo se la velocità è verso la parete, perché se no
   // dava luogo all'effetto di accelerazione infinita avanti e indietro. In
   // realtà questo potrebbe causare staticità perché se il vettore finale è 0
   // non viene ulteriormente aumentato ma non so come farlo senza una marea di
   // if
*/
int main() {
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "Boids Simulation");  // crea la finestra
  // window.setFramerateLimit(1000);                // numero di fps
  std::srand(
      std::time(nullptr));  // setta il seed della generazione casuale
                            // attraverso il tempo inizializzo la simulazione
  Params params{0.3, 0.8, 0.3, 45,
                20};  // parametri totalmente a caso, sono quasi sicuramente
                      // responsabili dello strano comportamento
  std::vector<Boid> stormo;
  for (int i = 0; i < 1000; ++i) {
    Boid boid({std::rand() % width, (std::rand() % height)},
              {((std::rand() % 4) - 2), ((std::rand() % 4) - 2)});

    stormo.push_back(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame
    for (auto& boid : stormo) {
      boid.update(params, stormo,
                  max_speed);  // funzione che limita la velocità

      draw(window, boid);

    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    window.display();
  }
  return 0;  // perché è un int main deve ritornare quando finisce
}