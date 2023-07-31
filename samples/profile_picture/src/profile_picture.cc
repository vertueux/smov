#include <profile_picture/profile_picture.h>
#include <iostream>

namespace smov {

void ProfilePicture::smooth_transition(float &receiver1, float &receiver2, float &receiver3, float &receiver4, float value) {
  // Lmao buddy thought we could divide by 0 haha lol ez xd.
  if (receiver1 == value) {
    std::cout << "Same values, not executing command lmao xd lol ez." << std::endl;
    return;
  }

  if (value > receiver1) {
    double delta = value - receiver1; // Différence entre les valeurs de receiver et value

    // Initialiser le temps
    int T = 4; // Temps en secondes
    int N = 100; // Nombre d'itérations
    int dt = T / N; // Intervalle de temps entre chaque itération

    // Boucle pour augmenter progressivement receiver à value
    for (int i = 0; i < N; i++) {
      // Calculer la nouvelle valeur de receiver
      receiver1 = receiver1 + delta / N;
      receiver2 = receiver2 + delta / N;
      receiver3 = receiver3 - delta / N;
      receiver4 = receiver4 - delta / N;


      front_state_publisher->publish(front_servos);
      back_state_publisher->publish(back_servos);


      // Afficher la valeur de receiver
      std::cout << "receiver: " << receiver3 << std::endl;

      // Attendre dt secondes
      std::this_thread::sleep_for(std::chrono::seconds(dt));
    }
  } else {
    double delta = receiver1 - value; // Différence entre les valeurs de receiver et value

    // Initialiser le temps
    int T = 4; // Temps en secondes
    int N = 100; // Nombre d'itérations
    int dt = T / N; // Intervalle de temps entre chaque itération

    // Boucle pour augmenter progressivement receiver à value
    for (int i = 0; i < N; i++) {
      // Calculer la nouvelle valeur de receiver
      receiver1 = receiver1 - delta / N;
      receiver2 = receiver2 - delta / N;
      receiver3 = receiver3 + delta / N;
      receiver4 = receiver4 + delta / N;

      front_state_publisher->publish(front_servos);
      back_state_publisher->publish(back_servos);

      // Afficher la valeur de receiver
      std::cout << "receiver: " << receiver3 << std::endl;

      // Attendre dt secondes
      std::this_thread::sleep_for(std::chrono::seconds(dt));
    }
  }

  std::cout << "---------------" << std::endl;
}

void ProfilePicture::animate_first_part() {
  smooth_transition(front_servos.value[LEFT_BODY], front_servos.value[RIGHT_BODY], 
    back_servos.value[LEFT_BODY] , back_servos.value[RIGHT_BODY], -0.5f);
  sleep(cooldown);

  // Transitioning to the second part.
  animate_second_part();
}

void ProfilePicture::animate_second_part() {
  smooth_transition(front_servos.value[LEFT_BODY], front_servos.value[RIGHT_BODY], 
    back_servos.value[LEFT_BODY] , back_servos.value[RIGHT_BODY], 0.5f);
  sleep(cooldown);
  
  // Transitioning to the first part.
  animate_first_part();
}

void ProfilePicture::on_start() {
  // Default value when awakened.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[i] = 0.0f;
    front_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;

    back_servos.value[i] = 0.0f;
    back_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }

  sleep(5);
  animate_first_part();
}

void ProfilePicture::on_loop() {
}

void ProfilePicture::on_quit() {
}

}

DECLARE_STATE_NODE_CLASS("smov_manual_walk_state", smov::ProfilePicture, 10ms)
