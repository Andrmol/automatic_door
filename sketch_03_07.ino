
//define pins

#define buffer_length (24)
#define blinking_duration (100) // Длительность моргания лампочек в миллисекундах.
#define blinking_frequency  (5) // Период моргания лампочек в миллисекундах.


#ifndef HIGH
#define HIGH 1
#define LOW  (!HIGH)
#endif

#define STOP_BUTTON_PIN 2

#define CLOSE_BUTTON_PIN A0
#define CLOSED_SWITCH_PIN 10
#define UNCLOSED_SWITCH_PIN 11

#define SEAL_BUTTON_PIN A1
#define SEALED_SWITCH_PIN 12

#define UNSEAL_BUTTON_PIN A2
#define UNSEALED_SWITCH_PIN 13

#define CLOSE_DIR_PIN 3
#define SEAL_DIR_PIN 5
#define CLOSE_ENBL_PIN 6
#define SEAL_ENBL_PIN 6
#define CLOSE_STEP_PIN 2
#define SEAL_STEP_PIN 4

#define CLOSE_LAMP_PIN 7
#define SEAL_LAMP_PIN 8
#define UNSEAL_LAMP_PIN 9

enum Engine_Item { CLOSE_ENG, SEAL_ENG, ENGINES_NUMBER};
enum Movements_Item { TO_CLOSE, TO_SEAL, TO_UNSEAL, TO_UNCLOSE, FROM_SEAL, FROM_UNSEAL, MOVEMENTS_NUMBER};

enum Scenario_Type {NONE, STARTUP, CLOSING, SEALING, UNSEALING, LAST_SCENARIO};

enum Scenario_State {
  CHECK_CLOSED,
  CHECK_UNSEALED,
  CHECK_SEALED,
  INIT_UNSEALED,
  WORK_UNSEALED,
  INIT_UNCLOSE,
  WORK_UNCLOSE,
  INIT_FROM_SEAL,
  WORK_FROM_SEAL,
  INIT_FROM_UNSEAL,
  WORK_FROM_UNSEAL,
  INIT_CLOSED,
  WORK_CLOSED,
  INIT_SEALED,
  WORK_SEALED,
  SCENARIO_EXIT,
  LAST_SC_STATE
};

enum Prim_State {
  WORKING,
  SUCCESS,
  STOPPED
};


typedef struct {   //структура управления двигателем
  const int dir_pin; //Пин сигнала direction
  const int step_pin; //Пин сигнала step
  const int v_start; //Стартовая скорость двигателя в шагах/с
  const int v_end; //Конечная скорость двигателя в шагах/с
  const int acc; // Ускорение двигателя в шагах/c^2
  const int enable_pin; // Пин сигнала enable
  const byte enable_pol; // Полярность сигнала enable
  const long timeout_uS; // Таймаут операции в микросекундах
} Engine ;

//инициализация двигателей
Engine engines[ENGINES_NUMBER] = {
  { //CLOSE_ENG
    .dir_pin = CLOSE_DIR_PIN,
    .step_pin = CLOSE_STEP_PIN,
    .v_start = 0,
    .v_end = 800,
    .acc = 16,
    .enable_pin = CLOSE_ENBL_PIN,
    .enable_pol = LOW,
    .timeout_uS = 20000000
  },
  { //SEAL_ENG
    .dir_pin = SEAL_DIR_PIN,
    .step_pin = SEAL_STEP_PIN,
    .v_start = 0,
    .v_end = 800,
    .acc = 16,
    .enable_pin = SEAL_ENBL_PIN,
    .enable_pol = LOW,
    .timeout_uS = 30000000
  }
};




typedef struct  {
  //управление движением в сторону концевика
  const int switch_pin; //пин концевика
  const byte switch_pol; //полярность успеха концевика
  const byte engine_dir_pol; //полярность направления движения мотора в сторону концевика
  Engine* engine; //указатель на структуру двигателя
} Movement;


//инициализация структур движений
Movement movements[MOVEMENTS_NUMBER] = {
  { //TO_CLOSE, 0
    .switch_pin = CLOSED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = HIGH,
    .engine = &engines[CLOSE_ENG]
  },
  { //TO_SEAL, 1
    .switch_pin = SEALED_SWITCH_PIN,
    .switch_pol = LOW,
    .engine_dir_pol = HIGH,
    .engine = &engines[SEAL_ENG]
  },
  { //TO_UNSEAL, 2
    .switch_pin = UNSEALED_SWITCH_PIN,
    .switch_pol = LOW,
    .engine_dir_pol = LOW,
    .engine = &engines[SEAL_ENG]
  },
  { //TO_UNCLOSE, 3
    .switch_pin = UNCLOSED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = LOW,
    .engine = &engines[CLOSE_ENG]
  },
  { //FROM_SEAL, 4
    .switch_pin = SEALED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = LOW,
    .engine = &engines[SEAL_ENG]
  },
  { //FROM_UNSEAL, 5
    .switch_pin = UNSEALED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = HIGH,
    .engine = &engines[SEAL_ENG]
  },
};

typedef struct {
  long accumulator;
  int accumulator_inc;
  long previous_activation_time;
  const int mask_step = 1 << buffer_length;
  bool previous_separator_bit_step;
} Accumulator_Context;

typedef struct  {
  bool active;

  int inc_vel_end;
  int current_inc_acc;
  bool step_event;
  long time_of_timeout;
  Movement* how_to_move;
  Prim_State prim_state;
  Accumulator_Context* accum_vel;
  Accumulator_Context* accum_acc;
} Prim_Context;


typedef struct {
  bool active;
  Prim_Context (*scenario_func)(Prim_Context, Movement);
  Scenario_State scenario_state;
  Scenario_Type scenario_type;
} Scen_Context;


typedef struct {
  //структура состояния кнопок и концевиков. активной может быть только одна кнопка. хранить состояние всех кнопок избыточно, достаточно иметь одну переменную хранящую активную кнопку.
  bool stop_state;
  bool close_switch_state;
  bool unclose_switch_state;
  bool close_button_state;
  bool seal_switch_state;
  bool seal_button_state;
  bool unseal_switch_state;
  bool unseal_button_state;
  bool close_light_blinking;
  long close_light_blinking_timeout;
  bool seal_light_blinking;
  long seal_light_blinking_timeout;
  bool unseal_light_blinking;
  long unseal_light_blinking_timeout;
} State;





typedef struct  {
  byte button_close_event;
  byte button_unseal_event;
  byte button_seal_event;
  byte button_stop_event;
  byte switch_close_event;
  byte switch_unclose_event;
  byte switch_seal_event;
  byte switch_unseal_event;
} Events;


void vel_accel (Prim_Context* prim_context); //Функция регулирующая запуск аккумуляторов. Генерирует step_event
void movement_to_switch(Prim_Context* prim_context, State state); //Абстрактный примитив движения в сторону выключателя. Осуществляет движение мотором
void init_primitive(Prim_Context* prim_context, Movement* movement); //Функция инициализирующая примитив и аккумуляторы.
void stop_primitive(Prim_Context* prim_context);

void init_primitive(Prim_Context* prim_context, Movement* movement) {

  double multiplier = (2l << buffer_length) / 1e6;
  prim_context->how_to_move = movement;
  prim_context->accum_vel->accumulator_inc = multiplier * movement->engine->v_start; //при инициализации задается равным inc_vel_start = length_of_buffer * 1 μs * engine.v_start
  prim_context->inc_vel_end = multiplier *  movement->engine->v_end; //при инициализации задается равным inc_vel_end = length_of_buffer * 1 μs * engine.v_end
  prim_context->accum_acc->accumulator_inc = multiplier  * movement->engine->acc  //при инициализации задается равным inc_acc = length_of_buffer * 1 μs * f_acc, где f_acc = Δinc / t_acc
      * (prim_context->inc_vel_end - prim_context->accum_vel->accumulator_inc)                //Δinc = inc_vel_end - inc_vel_start
      / ((movement->engine->v_end - movement->engine->v_start));                    //t_acc = (v_end - v_start)/ acc
  prim_context->time_of_timeout = micros() + movement->engine->timeout_uS;
  prim_context->active = true;
  prim_context->prim_state = WORKING;
  //Запись сигнала enable.
  digitalWrite(movement->engine->enable_pin, movement->engine->enable_pol);
  //Запись сигнала direction.
  digitalWrite(movement->engine->dir_pin, movement->engine_dir_pol);
}

void stop_primitive(Prim_Context* prim_context) {
  if (prim_context->how_to_move->engine->enable_pol == HIGH)
    digitalWrite(prim_context->how_to_move->engine->enable_pin, LOW);
  else
    digitalWrite(prim_context->how_to_move->engine->enable_pin, HIGH);
  prim_context->active = 0;
};


bool phase_accumulator (Accumulator_Context* accumulator_context) {
  //Фазовый аккумулятор регулирующий ход мотора.
  bool separator_bit;

  /*if (reset = true) { Более не требуется. Обнуление происходит инициализацией.
    accumulator_context->accumulator_step = 0;
    accumulator_context->previous_activation_time = 0;
    return false;
    }
    if (previous_activation_time == 0) {
    previous_activation_time = micros();
    }*/

  accumulator_context->accumulator +=  accumulator_context->accumulator_inc * (micros() - accumulator_context->previous_activation_time);
  accumulator_context->previous_activation_time = micros();
  separator_bit = (accumulator_context->accumulator & accumulator_context->mask_step) >> buffer_length;

  if (separator_bit != accumulator_context->previous_separator_bit_step) {
    accumulator_context->previous_separator_bit_step = separator_bit;
    return true;
  }

  return false;
}


void vel_accel (Prim_Context* prim_context) {
  if (!prim_context->active) return;
  /*if (reset) {
    phase_accumulator(prim_context->current_inc_acc);
    phase_accumulator(prim_context->current_inc_step;
    return prim_context;
    }*/
  //проверка аккумулятора ускорения. При успехе инкрементирует инкремент аккумулятора хода.
  if (prim_context->accum_vel->accumulator_inc < prim_context->inc_vel_end) {
    if (phase_accumulator(prim_context->accum_acc) == true) {
      prim_context->accum_vel->accumulator_inc++;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, при успехе создает событие step_event.
  if (phase_accumulator(prim_context->accum_vel) == true) {
    prim_context->step_event = true;
  }
}




void movement_to_switch(Prim_Context* prim_context, State* state) {
  //Абстрактный примитив первого уровня «Движение мотора W со скоростью X в направлении Y до срабатывания концевого выключателя Z».
  //Возвращает код результата работы. 0 - авария, 1 - успех, 2 - работа не закончена.

  if (prim_context->active == 0) {
    return;
  }
  //Проверка таймаута
  if (prim_context->time_of_timeout - micros() < 0) {
    prim_context->prim_state = STOPPED;
    stop_primitive(prim_context);
    return;
  }

  //Проверка состояния концевика.
  /* Проверка концевика вынесена в сценарий.
    if (digitalRead(prim_context->how_to_move->switch_pin) == prim_context->how_to_move->switch_pol)
    {
    prim_context->prim_state = SUCCESS;
    stop_primitive(prim_context);
    return;
    }
  */

  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.
  if (state->stop_state == true)
  {
    prim_context->prim_state = STOPPED;
    stop_primitive(prim_context);
    return;
  }

  if (prim_context->prim_state == WORKING) {
    if (prim_context->step_event) {
      prim_context->step_event = false;
      //Запись сигнала step обратного текущему.
      if (digitalRead(prim_context->how_to_move->engine->step_pin) == HIGH)
        digitalWrite(prim_context->how_to_move->engine->step_pin, LOW);
      else
        digitalWrite(prim_context->how_to_move->engine->step_pin, HIGH);
      //Запись сигнала step обратного текущему. Данное заклинание должно работать по причине указанной выше.
      //      digitalWrite(prim_context->how_to_move->engine->step_pin, !digitalRead(prim_context->how_to_move->engine->step_pin));
    }

    //  prim_context->prim_state = WORKING;
    return;
  }
}

void buttons_processing(State* state, Events* events, bool scenario_active) {
  static long signal_timeout_stop;
  static long signal_timeout_close;
  static long signal_timeout_unclose;
  static long signal_timeout_seal;
  static long signal_timeout_unseal;
  static long signal_timeout_close_button;
  static long signal_timeout_seal_button;
  static long signal_timeout_unseal_button;
  static const int delay_button = 20; //задержка прочтения нажатия кнопки, в миллисекундах
  static const int delay_switch = 10; //задержка прочтения сработавшего датчика, в миллисекундах
  //функция обработки нажатых кнопок с учетом дребезга. для кнопок задержка +- 20 милисекунд, для концевиков может быть меньше, +-10. Возвращает активный сценарий.

  //если есть активный сценарий, то даже не смотреть на кнопки
  /*
    if (digitalRead(STOP_BUTTON_PIN) == events->button_stop_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->button_stop_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (!state->stop_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_stop > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->stop_state = true;
          }
        }
      }
    }
    else {
      if (events->button_stop_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
        signal_timeout_stop = millis() + delay_switch;
        events->button_stop_event = HIGH;
      }
      else { //Если сигнал изменился с 1 на 0
        events->button_stop_event = LOW;
      }
    } */

  /*
    if (digitalRead(CLOSED_SWITCH_PIN) == events->switch_close_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->switch_close_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (!state->close_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_close > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->close_switch_state = true;
          }
        }
      }
      else { //Если текущее состояние совпадает с предыдущим и равно 0
        if (state->close_switch_state) {
          if (signal_timeout_close > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->close_switch_state = false;
          }
        }
      }
    }
    else {
      if (events->switch_close_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
        signal_timeout_close = millis() + delay_switch;
        events->switch_close_event = HIGH;
      }
      else { //Если сигнал изменился с 1 на 0
        signal_timeout_close = millis() + delay_switch;
        events->switch_close_event = LOW;
      }
    }


    if (digitalRead(UNCLOSED_SWITCH_PIN) == events->switch_unclose_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->switch_unclose_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (!state->unclose_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_unclose > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->unclose_switch_state = true;
          }
        }
      }
      else {
        if (state->unclose_switch_state) {
          if (signal_timeout_unclose > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->unclose_switch_state = false;
          }
        }
      }
    }

    else {
      if (events->switch_unclose_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
        signal_timeout_unclose = millis() + delay_switch;
        events->switch_unclose_event = HIGH;
      }
      else { //Если сигнал изменился с 1 на 0
        signal_timeout_unclose = millis() + delay_switch;
        events->switch_unclose_event = LOW;
      }
    } */

  if (digitalRead(CLOSED_SWITCH_PIN) == HIGH) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }

  if (digitalRead(UNCLOSED_SWITCH_PIN) == HIGH) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }

  if (digitalRead(SEALED_SWITCH_PIN) == HIGH) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }

  if (digitalRead(UNSEALED_SWITCH_PIN) == HIGH) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }
  /*
    if (digitalRead(SEALED_SWITCH_PIN) == events->switch_seal_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->switch_seal_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (!state->seal_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_seal > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->seal_switch_state = true;
          }
        }
      }
      else {
        if (state->seal_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_seal > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->seal_switch_state = false;
          }
        }
      }
    }
    else {
      if (events->switch_seal_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
        signal_timeout_seal = millis() + delay_switch;
        events->switch_seal_event = HIGH;
      }
      else { //Если сигнал изменился с 1 на 0
        events->switch_seal_event = LOW;
        signal_timeout_seal = millis() + delay_switch;
      }
    }*/


  /*
    if (digitalRead(UNSEALED_SWITCH_PIN) == events->switch_unseal_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->switch_unseal_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (!state->unseal_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_unseal > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->unseal_switch_state = true;
          }
        }
      }

      else {
        if (state->unseal_switch_state) { //Если датчик возвращает 1 и уже записан как 1, то нет смысла проверять таймер.
          if (signal_timeout_unseal > millis()) { //Если таймер прошел, то можно записать сигнал.
            state->unseal_switch_state = false;
          }
        }
      }
    }
    else {
      if (events->switch_unseal_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
        signal_timeout_unseal = millis() + delay_switch;
        events->switch_unseal_event = HIGH;
      }
      else { //Если сигнал изменился с 1 на 0
        events->switch_unseal_event = LOW;
      }
    }
  */

  if (scenario_active) {
    return;
  }

  if (digitalRead(CLOSE_BUTTON_PIN) == LOW) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }

  if (digitalRead(SEAL_BUTTON_PIN) == LOW) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }

  if (digitalRead(UNSEAL_BUTTON_PIN) == LOW) {
    state->seal_switch_state = true;
  }
  else {
    state->seal_switch_state = false;
  }
  /*
    if (!digitalRead(CLOSE_BUTTON_PIN) == events->button_close_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->button_close_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (signal_timeout_close_button > millis()) { //Если таймер прошел, то можно записать сигнал.
          state->close_button_state = true;
        }
      }
      else {
        if (events->button_close_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
          signal_timeout_close_button = millis() + delay_button;
          events->button_close_event = HIGH;
        }
        else { //Если сигнал изменился с 1 на 0
          events->button_close_event = LOW;
        }
      }
    }

    if (!digitalRead(SEAL_BUTTON_PIN) == events->button_seal_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->button_seal_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (signal_timeout_seal_button > millis()) { //Если таймер прошел, то можно записать сигнал.
          state->seal_button_state = true;
        }
      }
      else {
        if (events->button_seal_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
          signal_timeout_seal_button = millis() + delay_button;
          events->button_seal_event = HIGH;
        }
        else { //Если сигнал изменился с 1 на 0
          events->button_seal_event = LOW;
        }
      }
    }

    if (!digitalRead(UNSEAL_BUTTON_PIN) == events->button_unseal_event) { //Проверка текущего состояния с предыдущим записанным.
      if (events->button_unseal_event == HIGH) { //Если текущее состояние совпадает с предыдущим и равно 1, то проверить таймер
        if (signal_timeout_unseal_button > millis()) { //Если таймер прошел, то можно записать сигнал.
          state->unseal_button_state = true;
        }
      }
      else {
        if (events->button_unseal_event == LOW) { //Если сигнал изменился с 0 на 1, то запускаю таймер
          signal_timeout_unseal_button = millis() + delay_button;
          events->button_unseal_event = HIGH;
        }
        else { //Если сигнал изменился с 1 на 0
          events->button_unseal_event = LOW;
        }
      }
    }*/
}

void indication (State * state) {
  static long previous_close_blinking_time;
  static long current_close_blinking_time;
  static byte previous_close_blinking_state;

  static long previous_seal_blinking_time;
  static long current_seal_blinking_time;
  static byte previous_seal_blinking_state;

  static long previous_unseal_blinking_time;
  static long current_unseal_blinking_time;
  static byte previous_unseal_blinking_state;

  if (state->close_light_blinking) {
    current_close_blinking_time = millis();
    if (current_close_blinking_time > state->close_light_blinking_timeout) {
      state->close_light_blinking = false;
    }
    else {
      if (current_close_blinking_time - previous_close_blinking_time >= blinking_frequency) {
        if (previous_close_blinking_state == HIGH) {
          digitalWrite(CLOSE_LAMP_PIN, LOW);
          previous_close_blinking_state = LOW;
          previous_close_blinking_time = current_close_blinking_time;
        }
        else {
          digitalWrite(CLOSE_LAMP_PIN, HIGH);
          previous_close_blinking_state = HIGH;
          previous_close_blinking_time = current_close_blinking_time;
        }
      }
    }
  }
  else {
    if (state->close_switch_state) digitalWrite(CLOSE_LAMP_PIN, HIGH);
    else digitalWrite(CLOSE_LAMP_PIN, LOW);
  }

  if (state->seal_light_blinking) {
    current_seal_blinking_time = millis();
    if (current_seal_blinking_time > state->seal_light_blinking_timeout) {
      state->seal_light_blinking = false;
    }
    else {
      if (current_seal_blinking_time - previous_seal_blinking_time >= blinking_frequency) {
        if (previous_seal_blinking_state == HIGH) {
          digitalWrite(SEAL_LAMP_PIN, LOW);
          previous_seal_blinking_state = LOW;
          previous_seal_blinking_time = current_seal_blinking_time;
        }
        else {
          digitalWrite(SEAL_LAMP_PIN, HIGH);
          previous_seal_blinking_state = HIGH;
          previous_seal_blinking_time = current_seal_blinking_time;
        }
      }
    }
  }
  else {
    if (state->seal_switch_state) digitalWrite(SEAL_LAMP_PIN, HIGH);
    else digitalWrite(SEAL_LAMP_PIN, LOW);
  }


  if (state->unseal_light_blinking) {
    current_unseal_blinking_time = millis();
    if (current_unseal_blinking_time > state->unseal_light_blinking_timeout) {
      state->unseal_light_blinking = false;
    }
    else {
      if (current_unseal_blinking_time - previous_unseal_blinking_time >= blinking_frequency) {
        if (previous_unseal_blinking_state == HIGH) {
          digitalWrite(UNSEAL_LAMP_PIN, LOW);
          previous_unseal_blinking_state = LOW;
          previous_unseal_blinking_time = current_unseal_blinking_time;
        }
        else {
          digitalWrite(UNSEAL_LAMP_PIN, HIGH);
          previous_unseal_blinking_state = HIGH;
          previous_unseal_blinking_time = current_unseal_blinking_time;
        }
      }
    }
  }
  else {
    if (state->unseal_switch_state) digitalWrite(UNSEAL_LAMP_PIN, HIGH);
    else digitalWrite(UNSEAL_LAMP_PIN, LOW);
  }
}


void scenario_sealing(Prim_Context * prim_context, Scen_Context * scen_context, State * state, Movement movements[6]) {
  switch (scen_context->scenario_state)
  {
    case (CHECK_CLOSED):
      if (state->close_switch_state) {
        scen_context->scenario_state = INIT_SEALED;
        break;
      }
      scen_context->scenario_state = INIT_UNSEALED;
      break;
    case (INIT_UNSEALED):
      init_primitive(prim_context, &movements[2]);
      scen_context->scenario_state = WORK_SEALED;
      break;
    case (WORK_UNSEALED):
      if (state->unseal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_FROM_UNSEAL;
          break;
        default:
          break;
      }
    case (INIT_FROM_UNSEAL):
      init_primitive(prim_context, &movements[5]);
      scen_context->scenario_state = WORK_FROM_UNSEAL;
      break;
    case (WORK_FROM_UNSEAL):
      if (!state->unseal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_CLOSED;
          break;
        default:
          break;
      }
    case (INIT_CLOSED):
      init_primitive(prim_context, &movements[0]);
      scen_context->scenario_state = WORK_CLOSED;
      break;
    case (WORK_CLOSED):
      if (state->close_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_UNCLOSE;
          break;
        default:
          break;
      }
    case (INIT_UNCLOSE):
      {
        init_primitive(prim_context, &movements[3]);
        scen_context->scenario_state = WORK_UNCLOSE;
        break;
      }
    case (WORK_UNCLOSE):
      if (state->unclose_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_SEALED;
          break;
        default:
          break;
      }
    case (INIT_SEALED):
      init_primitive(prim_context, &movements[1]);
      scen_context->scenario_state = WORK_SEALED;
      break;
    case (WORK_SEALED):
      if (state->seal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_FROM_SEAL;
          break;
        default:
          break;
      }
    case (INIT_FROM_SEAL):
      init_primitive(prim_context, &movements[4]);
      scen_context->scenario_state = WORK_FROM_SEAL;
      break;
    case (WORK_FROM_SEAL):
      if (!state->seal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        default:
          break;
      }
    case (SCENARIO_EXIT):
      scen_context->scenario_state = LAST_SC_STATE;
      scen_context->scenario_type = LAST_SCENARIO;
      scen_context->active = false;
      state->seal_button_state = false;
      state->seal_light_blinking = true;
      state->seal_light_blinking_timeout = millis() + blinking_duration;
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}

void scenario_unsealing(Prim_Context * prim_context, Scen_Context * scen_context, State * state, Movement movements[6]) {
  int prim_result;
  switch (scen_context->scenario_state)
  {
    case (CHECK_UNSEALED):
      if (state->unseal_switch_state) {
        scen_context->scenario_state = SCENARIO_EXIT;
        break;
      }
      scen_context->scenario_state = INIT_UNSEALED;
      break;
    case (INIT_UNSEALED):
      init_primitive(prim_context, &movements[2]);
      scen_context->scenario_state = WORK_SEALED;
      break;
    case (WORK_UNSEALED):
      if (state->unseal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_FROM_UNSEAL;
          break;
        case WORKING:
          break;
        default:
          break;
      }
    case (INIT_FROM_UNSEAL):
      init_primitive(prim_context, &movements[5]);
      scen_context->scenario_state = WORK_FROM_UNSEAL;
      break;
    case (WORK_FROM_UNSEAL):
      if (!state->unseal_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        default:
          break;
      }
    case (SCENARIO_EXIT):
      scen_context->scenario_state = LAST_SC_STATE;
      scen_context->scenario_type = LAST_SCENARIO;
      scen_context->active = false;
      state->unseal_button_state = false;
      state->unseal_light_blinking = true;
      state->unseal_light_blinking_timeout = millis() + blinking_duration;
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}
void scenario_closing(Prim_Context * prim_context, Scen_Context * scen_context, State * state, Movement movements[6]) {

  switch (scen_context->scenario_state)
  {
    case (CHECK_CLOSED):
      if (state->close_switch_state) {
        scen_context->scenario_state = SCENARIO_EXIT;
        break;
      }
      scen_context->scenario_state = INIT_CLOSED;
    case (INIT_CLOSED):
      init_primitive(prim_context, &movements[0]);
      scen_context->scenario_state = WORK_CLOSED;
      break;
    case (WORK_CLOSED):
      if (state->close_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;// каша в голове
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = INIT_UNCLOSE;// каша в голове
          break;
        default:
          break;
      }
    case (INIT_UNCLOSE):
      {
        init_primitive(prim_context, &movements[3]);
        scen_context->scenario_state = WORK_UNCLOSE;
        break;
      }
    case (WORK_UNCLOSE):
      if (state->unclose_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        default:
          break;
      }
    case (SCENARIO_EXIT):
      scen_context->scenario_state = LAST_SC_STATE;
      scen_context->scenario_type = LAST_SCENARIO;
      scen_context->active = false;
      state->close_light_blinking = true;
      state->close_light_blinking_timeout = millis() + blinking_duration;
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}

void scenario_starup(Prim_Context * prim_context, Scen_Context * scen_context, State * state, Movement movements[6]) {
  switch (scen_context->scenario_state)
  {
    case (INIT_UNCLOSE):
      {
        init_primitive(prim_context, &movements[3]);
        scen_context->scenario_state = WORK_UNCLOSE;
        break;
      }
    case (WORK_UNCLOSE):
      if (state->unclose_switch_state) prim_context->prim_state = SUCCESS;
      switch (prim_context->prim_state)
      {
        case STOPPED: //отработка аварии выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        case SUCCESS: //отработка успеха выполнения примитива
          scen_context->scenario_state = SCENARIO_EXIT;
          break;
        default:
          break;
      }
    case (SCENARIO_EXIT):
      scen_context->scenario_state = LAST_SC_STATE;
      scen_context->scenario_type = LAST_SCENARIO;
      scen_context->active = false;
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}

Prim_Context prim_context;
Scen_Context scen_context;


State state;


Events events;




void setup() {
  //events.button_close_event = LOW;
  events.button_unseal_event = HIGH;
  events.button_seal_event = HIGH;
  events.button_stop_event = HIGH;
  events.switch_close_event = LOW;
  events.switch_unclose_event = LOW;
  events.switch_seal_event = LOW;
  events.switch_unseal_event = LOW;

  /*
    state.close_switch_state = false;
    state.seal_switch_state = false;
    state.unseal_switch_state = false;
  */

  Serial.begin(9600);

  //pinMode(STOP_BUTTON_PIN, INPUT);
  pinMode(CLOSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CLOSED_SWITCH_PIN, INPUT);
  pinMode(UNCLOSED_SWITCH_PIN, INPUT);
  pinMode(SEAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SEALED_SWITCH_PIN, INPUT);
  pinMode(UNSEAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(UNSEALED_SWITCH_PIN, INPUT);

  pinMode(CLOSE_DIR_PIN, OUTPUT);
  pinMode(SEAL_DIR_PIN, OUTPUT);
  pinMode(CLOSE_STEP_PIN, OUTPUT);
  pinMode(SEAL_STEP_PIN, OUTPUT);
  pinMode(CLOSE_ENBL_PIN, OUTPUT);

  pinMode(CLOSE_LAMP_PIN, OUTPUT);
  pinMode(SEAL_LAMP_PIN, OUTPUT);
  pinMode(UNSEAL_LAMP_PIN, OUTPUT);

  //При запуске вернуть двигатель закрытия в начальное положение
  scen_context.scenario_type = STARTUP;
  scen_context.scenario_state = INIT_UNCLOSE;
  scen_context.active = true;

}






//Инкремент аккумулятора шага. Изначально равен начальной скорости.
//int accumulator_inc_step = Engine.v_start;
void loop() {

  //обзвон кнопок и свичей. buttons_processing инициализирует сценарий при записи состояния кнопки в state
  buttons_processing(&state, &events, scen_context.active);
  indication (&state);


  vel_accel(&prim_context);

  movement_to_switch(&prim_context, &state);

  if (scen_context.active) {
    switch (scen_context.scenario_type)
    {
      case (STARTUP):
        scenario_starup(&prim_context, &scen_context, &state, movements);
        break;
      case (CLOSING):
        scenario_closing(&prim_context, &scen_context, &state, movements);
        break;
      case (SEALING):
        scenario_sealing(&prim_context, &scen_context, &state, movements);
        break;
      case (UNSEALING):
        scenario_unsealing(&prim_context, &scen_context, &state, movements);
        break;
      case (LAST_SCENARIO):
        break;
      default:
        break;
    }
  }

  else {
    if (state.close_button_state) {
      scen_context.scenario_type = CLOSING;
      scen_context.active = true;
    } else if (state.seal_button_state) {
      scen_context.scenario_type = SEALING;
      scen_context.active = true;
    } else if (state.unseal_button_state) {
      scen_context.scenario_type = UNSEALING;
      scen_context.active = true;
    }
  }
}
