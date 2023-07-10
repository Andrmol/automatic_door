﻿﻿//define pins
#define close_button_pin 1
#define stop_button_pin 2

enum switch_state {closed, sealed, unsealed};

enum scenario_state {check_closed, init_unsealed, work_unsealed, init_closed, work_closed, init_sealed, work_sealed, scenario_exit};

struct Events {
  bool button_close_event;
  bool button_unseal_event;
  bool button_seal_event; 
  bool button_stop_event;
  bool switch_stop_event;
  bool switch_seal_event;  
  bool switch_unseal_event;
};



struct Engine {
  //структура управления двигателем
  const int dir_pin; //Пин сигнала direction
  const int step_pin; //Пин сигнала step
  const int v_start; //Стартовая скорость двигателя
  const int v_end; //Конечная скорость двигателя
  const int acc; // Ускорение двигателя
  const int enable_pin; // Пин сигнала enable
  const byte enable_pol; // Полярность сигнала enable
  const int timeout; // Таймаут операции
};

struct Movement {
  //управление движением в сторону концевика
  const int switch_pin; //пин концевика
  const byte switch_pol; //полярность успеха концевика
  const byte engine_dir_pol; //полярность направления движения мотора в сторону концевика
  Engine* engine; //указатель на структуру двигателя
};


struct State {
  //структура состояния кнопок и концевиков. активной может быть только одна кнопка. хранить состояние всех кнопок избыточно, достаточно иметь одну переменную хранящую активную кнопку.
  bool stop_state;
  bool close_switch_state;
  bool close_button_state;
  bool seal_switch_state;
  bool seal_button_state;
  bool unseal_switch_state;
  bool unseal_button_state;
};

struct Prim_Context {
  bool active;
  int current_speed;
  int accumulator_inc_acc;
  int accumulator_inc_step;
  Movement* movement;
  bool step_event;
  int time_of_start;
};

//TODO структура управления движением от концевки до его выключения

struct Scen_Context {
  bool active;
  int (*scenario_func)();
  int scenario_state;
};

bool phase_accumulator_step (int accumulator_inc_step, bool reset = false) {
  //Фазовый аккумулятор регулирующий ход мотора. Начальный инкремент соответствует начальной скорости, увеличивается аккумулятором ускорения.
  static long accumulator_step;
  static const int buffer_length_step = 16;
  static const int mask_step = 1 << buffer_length_step;
  static bool previous_separator_bit_step;
  bool separator_bit;

  if (reset = true) {
    accumulator_step = 0;
    return false;
  }

  accumulator_step += accumulator_inc_step;

  separator_bit = (accumulator_step & mask_step) >> buffer_length_step;

  if (separator_bit != previous_separator_bit_step) {
    previous_separator_bit_step = separator_bit;
    return true;
  }

  return false;

}

bool phase_accumulator_acc (bool reset = false) {
  //Фазовый аккумулятор регулирующий ускорение. Величина инкремента постоянна и высчитывается двумя способами:
  //Если инкремент соответствующий начальной скорости мотора незначительно больше инкремента ускорения, то можно использовать accumulator_inc_acc = Engine.v_start
  //Иначе его можно вычислить как (Engine.v_end - Engine.v_start)/t, где t - время ускорения от начальной до конечной скорости.

  static const int accumulator_inc_acc;
  static long accumulator_acc;
  static const int buffer_length_acc = 16;
  static const int mask_acc = 1 << buffer_length_acc;
  bool separator_bit;
  static bool previous_separator_bit_acc;


  if (reset == true) {
    accumulator_acc = 0;
    return false;
  }

  accumulator_acc += accumulator_inc_acc;

  separator_bit = (accumulator_acc & mask_acc) >> buffer_length_acc;

  if (separator_bit != previous_separator_bit_acc) {
    previous_separator_bit_acc = separator_bit;
    return true;
  }

  return false;

}

int movement_to_switch(Prim_Context prim_context) {
  //Абстрактный примитив первого уровня «Движение мотора W со скоростью X в направлении Y до срабатывания концевого выключателя Z».
  //Возвращает код результата работы. 0 - авария, 1 - успех, 2 - работа не закончена.

  //Запуск счетчика общего таймаута.
  if (prim_context.time_of_start != 0)
  {
    prim_context.time_of_start = micros();
    //Запись сигнала enable. Таким образом будет вызываться только при первом запуске функции. Предположительно существует более красивое решение.
    digitalWrite(prim_context.movement->engine->enable_pin, prim_context.movement->engine->enable_pol);
  }

  //Проверка состояния концевика.
  //TODO вынести проверку концевиков в отдельную функцию.
  if (digitalRead(prim_context.movement.switch_pin) == prim.context.movement.switch_pol)
  {
    time_of_start = 0;
    //Ардуино хранит цифровые сигналы не как булеву переменную, а как значения HIGH и LOW. Чему равны эти значения зависит от имплементации, логическое отрицание
    //записанного в переменную значения работать не будет. Но !digitalRead(pin) должно работать и инвертировать полученное значение.
    digitalWrite(prim_context.movement->engine->enable_pin, LOW);
    return 1;
  }


  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.
  //TODO понять как происходит обработка стопстейт
  /*
  if (stop_state == true)
  {
    time_of_start = 0;
    digitalWrite(movement.engine->enable_pin, LOW);
    return 0;
  }
  */
  //Запись сигнала direction.
  digitalWrite(movement.engine->dir_pin, movement.engine_dir_pol);

  //Запись сигнала step обратного текущему. Данное заклинание должно работать по причине указанной выше.
  digitalWrite(movement.engine->step_pin, !digitalRead(movement.engine->step_pin));

  //Проверка таймаута.
  if (micros() - prim_context.time_of_start < prim_context.movement->engine->timeout)
  {
    prim_context.time_of_start = 0;
    digitalWrite(prim_context.movement->engine->enable_pin, LOW);
    return 0;
  }

  return 2;
}


State buttons_processing(State state, Events events) {
  //TODO переписать на работу со структурами
  static int last_signal_time = 2147483647;
  static const int delay_button;
  //функция обработки нажатых кнопок с учетом дребезга. для кнопок задержка +- 20 милисекунд, для концевиков может быть меньше, +-10. Возвращает активный сценарий.

  //если есть активный сценарий, то даже не смотреть на кнопки
  if (active_scenario != 0) {
    return active_scenario;
  }

  //повторить для всех трех кнопок
  if (digitalRead(close_button_pin) == HIGH) {
    if (millis() - last_signal_time > delay_button) {
      last_signal_time = 2147483647;
      return 1;
    }
    else {
      last_signal_time = millis();
    }

  }
  return state;
}

bool stop_button_processing(bool stop_state) {
  static int last_signal_time = 2147483647;
  static const int delay_button;
  //функция обработки кнопки стоп.

  if (stop_state) {
    return stop_state;
  }

  //повторить для всех трех кнопок
  if (digitalRead(stop_button_pin) == HIGH) {
    if (millis() - last_signal_time > delay_button) {
      last_signal_time = 2147483647;
      return true;
    }
    else {
      last_signal_time = millis();
      return false;
    }

  }
}


int scenario_close (State state, Movement movement) {

}


Prim_Context vel_accel (Prim_Context prim_context, bool reset = false) {
  if (prim_context.current_speed < prim_context.movement->engine->v_end) {
    if (phase_accumulator_acc() == true) {
      prim_context.current_speed++;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, при успехе создает событие step_event
  if (phase_accumulator_step(prim_context.accumulator_inc_step) == true) {
    prim_context.step_event = true;
  }
  return prim_context;
}
int scenario_unseal (State state, Prim_Context prim_context) {
    
}


/* TODO сделать это
int scenario_manager(Prim_Context prim_context, Scen_Context scen_context, State state) {
  //функция запуска и контроля статусов выполнения сценариев.
  static int scenario_state;

  switch (active_scenario)
  {
    case closed:
      scenario_state = scenario_close(state, movement);
      return scenario_state;
    case unsealed:
      scenario_state = scenario_unsealed(state, movement);
      return scenario_state;
    case sealed:
      scenario_state = scenario_seal (state, movement);
      return scenario_state;
    default:
      return scenario_state;
  }

*/
  void setup() {

    Serial.begin(9600);
    //define pin modes

  }


  State state;
  Prim_Context prim_context;
  Scen_Context scen_context;
  Events events;
  int active_scenario;
  int scenario_state;

  //Инкремент аккумулятора шага. Изначально равен начальной скорости.
  //int accumulator_inc_step = Engine.v_start;
  void loop() {

    //обзвон кнопок и свичей
    state = buttons_processing(state, events);
    state.stop_state = stop_button_processing(state.stop_state);

    //TODO добавить функцию приведения состояния лампочек в соответствие state

    prim_context = vel_accel(prim_context);

    //TODO модерирует prim_context
    //prim_context = scenario_manager(prim_context, scen_context, state);

    if (prim_context.step_signal){
      movement_to_switch(prim_context, state);
      prim_context.step_signal = false;
    }
/*
    //запуск обработки сценариев
    scenario_state = scenario_manager(state, active_scenario);
    switch (scenario_state)
    {
      case 1:
        active_scenario = 0;
        state.stop_state = false;
        break;
      case 2: 
        active_scenario = 0;
        state.stop_state = false;
        break;
      default:
        break;
    }
*/
  }