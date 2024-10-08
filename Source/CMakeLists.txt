cmake_minimum_required(VERSION 3.16)
project(Project_PAXO VERSION 1.0.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast")

# abilita asserzioni di debug (in gcc), l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -D_GLIBCXX_ASSERTIONS -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

# richiedi il componente graphics della libreria SFML, versione 2.5
#   le dipendenze vengono identificate automaticamente
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

add_executable(Project_PAXO main.cpp Boids.cpp IOhandling.cpp)
target_link_libraries(Project_PAXO PRIVATE sfml-graphics)

# se il testing e' abilitato...
#   per disabilitare il testing, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)
    add_executable(Boid_test Boidtest.cpp Boids.cpp IOhandling.cpp)
    target_link_libraries(Boid_test PRIVATE sfml-graphics)
    add_test(NAME Boid_test COMMAND Boid_test)  
# aggiungi l'eseguibile all.t
  # add_executable(all.t all.t.cpp complex.t.cpp)
  # aggiungi l'eseguibile all.t alla lista dei test
  # add_test(NAME all.t COMMAND all.t)

endif()
