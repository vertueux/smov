#include <manual_walk/manual_walk.h>
#include <iostream>

namespace smov {

void ManualWalk::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars); /* grab old terminal i/o settings */
  new_chars = old_chars; /* make new settings same as old settings */
  new_chars.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_chars); /* use these new terminal i/o settings now */
}

void ManualWalk::smooth_transition(float &receiver, float value) {
  // Lmao buddy thought we could divide by 0 haha lol ez xd.
  if (receiver == value) {
    std::cout << "Same values, not executing command lmao xd lol ez." << std::endl;
    return;
  }

  if (value > receiver) {
    double delta = value - receiver; // Différence entre les valeurs de receiver et value

    // Initialiser le temps
    int T = 2; // Temps en secondes
    int N = 100; // Nombre d'itérations
    int dt = T / N; // Intervalle de temps entre chaque itération

    // Boucle pour augmenter progressivement receiver à value
    for (int i = 0; i < N; i++) {
      // Calculer la nouvelle valeur de receiver
      receiver = receiver + delta / N;

      front_state_publisher->publish(front_servos);
      back_state_publisher->publish(back_servos);


      // Afficher la valeur de receiver
      std::cout << "receiver: " << receiver << std::endl;

      // Attendre dt secondes
      std::this_thread::sleep_for(std::chrono::seconds(dt));
    }
  } else {
    double delta = receiver - value; // Différence entre les valeurs de receiver et value

    // Initialiser le temps
    int T = 2; // Temps en secondes
    int N = 100; // Nombre d'itérations
    int dt = T / N; // Intervalle de temps entre chaque itération

    // Boucle pour augmenter progressivement receiver à value
    for (int i = 0; i < N; i++) {
      // Calculer la nouvelle valeur de receiver
      receiver = receiver - delta / N;

      front_state_publisher->publish(front_servos);
      back_state_publisher->publish(back_servos);

      // Afficher la valeur de receiver
      std::cout << "receiver: " << receiver << std::endl;

      // Attendre dt secondes
      std::this_thread::sleep_for(std::chrono::seconds(dt));
    }
  }

  std::cout << "---------------" << std::endl;
}

void ManualWalk::execute_forward_sequence() {

  /*smooth_transition(front_servos.value[RIGHT_BICEPS], -0.066f);
  smooth_transition(back_servos.value[LEFT_BICEPS], -0.066f);*/
  smooth_transition(front_servos.value[LEFT_BICEPS], -0.2f);
  smooth_transition(back_servos.value[RIGHT_BICEPS], -0.2f);

  std::cout << "========Cooldown=========" << "F Val:" << front_servos.value[LEFT_BICEPS] << std::endl;
  sleep(cooldown);

  smooth_transition(front_servos.value[LEFT_BICEPS], -0.066f);
  smooth_transition(back_servos.value[RIGHT_BICEPS], -0.066f);
  smooth_transition(front_servos.value[RIGHT_BICEPS], -0.2f);
  smooth_transition(back_servos.value[LEFT_BICEPS], -0.2f);
  sleep(cooldown);
}

void ManualWalk::execute_backward_sequence() {
  
}

void ManualWalk::execute_right_sequence() {
  
}

void ManualWalk::execute_left_sequence() {
  
}

void ManualWalk::on_start() {
  //init_reader(0);

  // Default value when awakened.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[i] = 0.0f;
    front_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;

    back_servos.value[i] = 0.0f;
    back_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }

  execute_forward_sequence();
}

void ManualWalk::on_loop() {
  /*int cmd = getchar();

  switch (cmd) {
    case KEY_UP:
      request_front_walk = true;
      break;
    case KEY_DOWN:
      request_back_walk = true;
      break;
    case KEY_RIGHT:
      request_right_walk = true;
      break;
    case KEY_LEFT:
      request_left_walk = true;
      break;
  }

  if (request_front_walk == true) 
    execute_forward_sequence();
  else if (request_back_walk == true) 
    execute_backward_sequence();
  else if (request_right_walk == true) 
    execute_right_sequence();
  else if (request_left_walk == true)
    execute_left_sequence();*/
}

void ManualWalk::on_quit() {
  //tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

}

DECLARE_STATE_NODE_CLASS("smov_manual_walk_state", smov::ManualWalk, 10ms)
