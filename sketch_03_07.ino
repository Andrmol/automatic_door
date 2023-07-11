﻿//define pins
#define close_button_pin 1
#define stop_button_pin 2
#define seal_button_pin 3
#define unseal_button_pin 4
#define close_switch_pin 3
#define seal_switch_pin 3
#define unseal_switch_pin 3



enum Scenario_Type {closing, sealing, unsealing, none};

enum Scenario_State {check_closed, check_unsealed, check_sealed, init_unsealed, work_unsealed, init_closed, work_closed, init_sealed, work_sealed, scenario_exit, none};


struct Engine {
  //структура управления двигателем
  const int dir_pin; //Пин сигнала direction
  const int step_pin; //Пин сигнала step
  const int v_start; //Стартовая скорость двигателя в шагах/с
  const int v_end; //Конечная скорость двигателя в шагах/с
  const int acc; // Ускорение двигателя в шагах/c^2
  const int enable_pin; // Пин сигнала enable
  const byte enable_pol; // Полярность сигнала enable
  const long timeout; // Таймаут операции в микросекундах
};
//инициализация двигателей
Engine engine_closing;
//engine_closing.dir_pin = smth;
//engine_closing.step_pin = smth;
engine_closing.v_start = 0;
engine_closing.v_end = 800;
engine_closing.acc = 16;
//engine_closing.enable_pin = smth;
//engine_closing.enable_pol = smth;
engine_closing.timeout = 10000;

Engine engine_sealing;
//engine_sealing.dir_pin = smth;
//engine_sealing.step_pin = smth;
engine_sealing.v_start = 0;
engine_sealing.v_end = 800;
engine_sealing.acc = 16;
//engine_sealing.enable_pin = smth;
//engine_sealing.enable_pol = smth;
engine_sealing.timeout = 10000;



struct Movement {
  //управление движением в сторону концевика
  const int switch_pin; //пин концевика
  const byte switch_pol; //полярность успеха концевика
  const byte engine_dir_pol; //полярность направления движения мотора в сторону концевика
  Engine* engine; //указатель на структуру двигателя
};


//инициализация структур концевиков. 0 - концевик "Закрыто", 1 - концевик "Задраено", 2 - концевик "Отдраено"
Movement movements[3];

//movements[0].switch_pin = smth;
movements[0].switch_pin = HIGH;
movements[0].switch_pin = HIGH;
movemnts[0].engine = &engine_closing;

//movements[1].switch_pin = smth;
movements[1].switch_pin = HIGH;
movements[1].switch_pin = HIGH;
movemnts[1].engine = &engine_sealing;

//movements[2].switch_pin = smth;
movements[2].switch_pin = HIGH;
movements[2].switch_pin = LOW;
movemnts[2].engine = &engine_sealing;


struct State {
  //структура состояния кнопок и концевиков. активной может быть только одна кнопка. хранить состояние всех кнопок избыточно, достаточно иметь одну переменную хранящую активную кнопку.
  bool stop_state;
  bool close_switch_state;
  bool close_button_state;
  bool seal_switch_state;
  bool seal_button_state;
  bool unseal_switch_state;
  bool unseal_button_state;
  Scen_Context* scen_context; //Указатель на контекст сценариев. Нужен чтобы инициализировать сценарий при нажатии на кнопку.
};

struct Prim_Context {
  bool active;
  int current_inc_step; //при инициализации задается равным inc_vel_start = length_of_buffer * 1 μs * engine.v_start
  int inc_vel_end; //при инициализации задается равным inc_vel_end = length_of_buffer * 1 μs * engine.v_end
  int current_inc_acc; //при инициализации задается равным inc_acc = length_of_buffer * 1 μs * f_acc, где f_acc = Δinc / t_acc
                       //Δinc = inc_vel_end - inc_vel_start
                       //t_acc = (v_end - v_start)/ acc
  bool step_event;
  long time_of_timeout;
};


//TODO структура управления движением от концевки до его выключения

struct Scen_Context {
  bool active;
  Prim_Context (*scenario_func)(Prim_Context, Movement);
  Scenario_State scenario_state;
  Scenario_Type scenario_type;
};


struct Events {
  bool button_close_event;
  bool button_unseal_event;
  bool button_seal_event;
  bool button_stop_event;
  bool switch_stop_event;
  bool switch_seal_event;
  bool switch_unseal_event;
};


bool phase_accumulator_step (int accumulator_inc_step, bool reset = false) {
  //Фазовый аккумулятор регулирующий ход мотора. Начальный инкремент соответствует начальной скорости, увеличивается аккумулятором ускорения.
  static long accumulator_step;
  static long previous_activation_time = micros();
  static const int buffer_length_step = 24;
  static const int mask_step = 1 << buffer_length_step;
  static bool previous_separator_bit_step;
  bool separator_bit;

  if (reset = true) {
    accumulator_step = 0;
    previous_activation_time = 0;
    return false;
  }
  if (previous_activation_time == 0){previous_activation_time = micros()}
  
  accumulator_step +=  accumulator_inc_step * (micros() - previous_activation_time);
  previous_activation_time = micros();
  separator_bit = (accumulator_step & mask_step) >> buffer_length_step;

  if (separator_bit != previous_separator_bit_step) {
    previous_separator_bit_step = separator_bit;
    return true;
  }

  return false;

}

bool phase_accumulator_acc (int accumulator_inc_acc, bool reset = false) {
  //Фазовый аккумулятор регулирующий ускорение. Величина инкремента постоянна и высчитывается двумя способами:
  //Если инкремент соответствующий начальной скорости мотора незначительно больше инкремента ускорения, то можно использовать accumulator_inc_acc = Engine.v_start
  //Иначе его можно вычислить как (Engine.v_end - Engine.v_start)/t, где t - время ускорения от начальной до конечной скорости.

  static long accumulator_acc;
  static long previous_activation_time = micros();
  static const int buffer_length_acc = 24;
  static const int mask_acc = 1 << buffer_length_acc;
  bool separator_bit;
  static bool previous_separator_bit_acc;


  if (reset == true) {
    accumulator_acc = 0;
    previous_activation_time = 0;
    return false;
  }

  if (previous_activation_time == 0){previous_activation_time = micros()}

  accumulator_acc += accumulator_inc_acc  * (micros() - previous_activation_time);
  previous_activation_time = micros();

  separator_bit = (accumulator_acc & mask_acc) >> buffer_length_acc;

  if (separator_bit != previous_separator_bit_acc) {
    previous_separator_bit_acc = separator_bit;
    return true;
  }

  return false;

}


Prim_Context vel_accel (Prim_Context prim_context, bool reset = false) {
  if (reset){
    phase_accumulator_acc(prim_context.current_inc_acc, true);
    phase_accumulator_step(prim_context.current_inc_step, true);
    return prim_context;  
  }
  
  if (prim_context.current_inc_step < prim_context.inc_vel_end) {
    if (phase_accumulator_acc(prim_context.current_inc_acc) == true) {
      prim_context.current_inc_step++;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, при успехе создает событие step_event
  if (phase_accumulator_step(prim_context.current_inc_step) == true) {
    prim_context.step_event = true;
  }
  return prim_context;
}




int movement_to_switch(Prim_Context prim_context, State state, Movement movement) {
  //Абстрактный примитив первого уровня «Движение мотора W со скоростью X в направлении Y до срабатывания концевого выключателя Z».
  //Возвращает код результата работы. 0 - авария, 1 - успех, 2 - работа не закончена.

  //Проверка таймаута
  if (prim_context.time_of_timeout - micros() < 0) {
    return 0;
  }

  //Проверка состояния концевика.
  //TODO вынести проверку концевиков в отдельную функцию.
  if (digitalRead(movement.switch_pin) == movement.switch_pol)
  {
    //Ардуино хранит цифровые сигналы не как булеву переменную, а как значения HIGH и LOW. Чему равны эти значения зависит от имплементации, логическое отрицание
    //записанного в переменную значения работать не будет. Но !digitalRead(pin) должно работать и инвертировать полученное значение.
    digitalWrite(prim_context.movement->engine->enable_pin, LOW);
    return 1;
  }


  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.

  if (state.stop_state == true)
  {
    digitalWrite(movement.engine->enable_pin, LOW);
    return 0;
  }

  //Запись сигнала direction.
  digitalWrite(movement.engine->dir_pin, movement.engine_dir_pol);

  //Запись сигнала step обратного текущему. Данное заклинание должно работать по причине указанной выше.
  digitalWrite(movement.engine->step_pin, !digitalRead(movement.engine->step_pin));

  return 2;
}


State buttons_processing(State state, Events events, bool scenario_active) {
  //TODO переписать на работу со структурами
  static int last_signal_time_stop = 2147483646;
  static int last_signal_time_seal = 2147483646;
  static int last_signal_time_close = 2147483646;
  static int last_signal_time_unseal = 2147483646;
  static int last_signal_time_unseal_sw = 2147483646;
  static int last_signal_time_seal_sw = 2147483646;
  static int last_signal_time_stop_sw = 2147483646;
  static const int delay_button; //задержка прочтения нажатия кнопки, в микросекундах
  static const int delay_switch;
  //функция обработки нажатых кнопок с учетом дребезга. для кнопок задержка +- 20 милисекунд, для концевиков может быть меньше, +-10. Возвращает активный сценарий.

  //если есть активный сценарий, то даже не смотреть на кнопки

  //TODO переписать на events, разобраться как учитывать время. Можно хранить время для каждого сигнала, но это звучит избыточно, возможно существует решение лучше.

  if (digitalRead(stop_button_pin) == HIGH) {
    if (micros() - last_signal_time_stop > delay_button) {
      last_signal_time_stop = 2147483646;
      state.stop_state = true;
    }
    else {
      last_signal_time_stop = micros();
    }

  if (digitalRead(stop_switch_pin) == HIGH) {
    if (micros() - last_signal_time_stop_sw > delay_button) {
      last_signal_time_stop_sw = 2147483646;
      state.stop_switch_state = true;
    }
    else {
      last_signal_time_stop_sw = micros();
    }

  }

  if (digitalRead(seal_switch_pin) == HIGH) {
    if (micros() - last_signal_time_seal_sw > delay_button) {
      last_signal_time_seal_sw = 2147483646;
      state.seal_switch_state = true;
    }
    else {
      last_signal_time_seal_sw = micros();
    }

  }


  if (digitalRead(unseal_switch_pin) == HIGH) {
    if (micros() - last_signal_time_unseal_sw > delay_button) {
      last_signal_time_unseal_sw = 2147483646;
      state.unseal_switch_state = true;
    }
    else {
      last_signal_time_unseal_sw = micros();
    }

  }
  
  if (scenario_active) {
    return state;
  }

  if (digitalRead(close_button_pin) == HIGH) {
    if (micros() - last_signal_time_close > delay_button) {
      last_signal_time = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = closing;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_close = micros();
    }

    if (digitalRead(seal_button_pin) == HIGH) {
    if (micros() - last_signal_time_seal > delay_button) {
      last_signal_time = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = sealing;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_seal = micros();
    }

  if (digitalRead(unseal_button_pin) == HIGH) {
    if (micros() - last_signal_time_unseal > delay_button) {
      last_signal_time = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = unsealing;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_unseal = micros();
    }

  }
  return state;
}

Prim_Context init_primitive(Prim_Context prim_context, Movement movement) {
  prim_context.current_inc_step = 16 * movement.engine->v_start;
  prim_context.inc_vel_end = 16 *  movement.engine->v_end;
  prim_context.current_inc_acc = (16 * (prim_context.inc_vel_end - prime_context.current_inc_step)) / ((movement.engine->v_end - movement.engine->v_start) / movement.engine->acc);
  prim_context.time_of_timeout = micros() + movement.engine->timeout;

  //Запись сигнала enable.
  digitalWrite(movement.engine->enable_pin, movement.engine->enable_pol);

  return prim_context;
}



Scen_Context scenario_sealing(Prim_Context prim_context, Scen_Context scen_context, State state) {
   int prim_result;
  switch (scen_context.scenario_state)
  {
    case (check_closed):
      if(*state.closed_switch_state){
        scen_context.scenario_state = init_sealed;
        return scen_context;
        }
      scen_context.scenario_state = init_unsealed;
      return scen_context;
    case (init_unsealed):
      *prim_context = init_primitive(*prim_context, movements[2]);
      scen_context.scenario_state = work_sealed;
      return scen_context;
    case (work_unsealed):
      if (*prim_context.step_signal) {
        prim_result = movement_to_switch(*prim_context, state);
        *prim_context.step_signal = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = init_closed;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (init_closed):
      *prim_context = init_primitive(*prim_context, movements[0]);
      scen_context.scenario_state = work_closed;
      return scen_context;
    case (work_closed):
      if (*prim_context.step_signal) {
        prim_result = movement_to_switch(*prim_context, state);
        *prim_context.step_signal = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = init_sealed;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (init_sealed):
      *prim_context = init_primitive(*prim_context, movements[1]);
      scen_context.scenario_state = work_sealed;
      return scen_context;
    case (work_sealed):
      if (*prim_context.step_signal) {
        prim_result = movement_to_switch(*prim_context, state);
        *prim_context.step_signal = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (scenario_exit):
      scen_context.scenario_state = none;
      scen_context.scenario_type = none;
      scen_context.active = false;
      return scen_context;
    case (none):
      return scen_context;
    default:
      return scen_context;
  }
}

Scen_Context scenario_unsealing(Prim_Context prim_context, Scen_Context scen_context, State state) {
   int prim_result;
  switch (scen_context.scenario_state)
  {
    case (check_unsealed):
      if(*state.unseal_switch_state){
        scen_context.scenario_state = scenario_exit;
        return scen_context;
        }
      scen_context.scenario_state = init_unsealed;
      return scen_context;
    case (init_unsealed):
      *prim_context = init_primitive(*prim_context, movements[2]);
      scen_context.scenario_state = work_sealed;
      return scen_context;
    case (work_unsealed):
      if (*prim_context.step_signal) {
        prim_result = movement_to_switch(*prim_context, state);
        *prim_context.step_signal = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (scenario_exit):
      scen_context.scenario_state = none;
      scen_context.scenario_type = none;
      scen_context.active = false;
      return scen_context;
    case (none):
      return scen_context;
    default:
      return scen_context;
  }
}

Scen_Context scenario_closing(Prim_Context* prim_context, Scen_Context scen_context, State* state, Movement movements[3]) {
  int prim_result;
  switch (scen_context.scenario_state)
  {
    case (check_closed):
      if(*state.close_switch_state){
        scen_context.scenario_state = scenario_exit;
        return scen_context;
        }
      scen_context.scenario_state = init_closed;
      return scen_context;
    case (init_closed):
      *prim_context = init_primitive(*prim_context, movements[0]);
      scen_context.scenario_state = work_closed;
      return scen_context;
    case (work_closed):
      if (*prim_context.step_signal) {
        prim_result = movement_to_switch(*prim_context, state);
        *prim_context.step_signal = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          *state.stop_state = false;
          scen_context.scenario_state = scenario_exit;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (scenario_exit):
      scen_context.scenario_state = none;
      scen_context.scenario_type = none;
      scen_context.active = false;
      return scen_context;
    case (none):
      return scen_context;
    default:
      return scen_context;
  }
}





void setup() {

  Serial.begin(9600);
  //define pin modes

}


Prim_Context prim_context;

Scen_Context scen_context;
scen_context.scenario_type = none;
scen_context.scenario_state = none;

State state;
state.scen_context = &scen_context;

Events events;





//Инкремент аккумулятора шага. Изначально равен начальной скорости.
//int accumulator_inc_step = Engine.v_start;
void loop() {

  //обзвон кнопок и свичей. buttons_processing инициализирует сценарий при записи состояния кнопки в state
  state = buttons_processing(state, events, scen_context.active);

  //TODO добавить функцию приведения состояния лампочек в соответствие state

  prim_context = vel_accel(prim_context);

  switch (scen_context.scenario_type)
  {
    case (closing):
      scen_context = scenario_closing(&prim_context, scen_context, &state, movements);
      break;
    case (sealing):
      scen_context = scenario_sealing(&prim_context, scen_context, &state, movements);
      break;
    case (unsealing):
      scen_context = scenario_unsealing(&prim_context, scen_context, &state, movements);
      break;
    case (none):
      break;
  }


}