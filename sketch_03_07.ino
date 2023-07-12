//define pins

#ifndef HIGH
#define HIGH 1
#define LOW  (!HIGH)
#endif

#define STOP_BUTTON_PIN 2

#define CLOSE_BUTTON_PIN 1
#define CLOSED_SWITCH_PIN 3
#define OPENED_SWITCH_PIN 3

#define SEAL_BUTTON_PIN 3
#define SEALED_SWITCH_PIN 3

#define UNSEAL_BUTTON_PIN 4
#define UNSEALED_SWITCH_PIN 4

#define CLOSE_DIR_PIN 3
#define SEAL_DIR_PIN 3
#define CLOSE_ENBL_PIN 3
#define SEAL_ENBL_PIN 3
#define CLOSE_STEP_PIN 3
#define SEAL_STEP_PIN 3

enum Engine_Item { CLOSE_ENG, SEAL_ENG, ENGINES_NUMBER};
enum Movements_Item { TO_CLOSE, TO_SEAL, TO_UNSEAL, FROM_SEAL, FROM_UNSEAL, MOVEMENTS_NUMBER};

enum Scenario_Type {CLOSING, SEALING, UNSEALING, LAST_SCENARIO};

enum Scenario_State {
  CHECK_CLOSED,
  CHECK_UNSEALED,
  CHECK_SEALED,
  INIT_UNSEALED,
  WORK_UNSEALED,
  INIT_CLOSED,
  WORK_CLOSED,
  INIT_SEALED,
  WORK_SEALED,
  SCENARIO_EXIT,
  LAST_SC_STATE
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
    .enable_pol = HIGH,
    .timeout_uS = 20000000L
  },
  { //SEAL_ENG
    .dir_pin = SEAL_DIR_PIN,
    .step_pin = SEAL_STEP_PIN,
    .v_start = 0,
    .v_end = 800,
    .acc = 16,
    .enable_pin = SEAL_ENBL_PIN,
    .enable_pol = LOW,
    .timeout_uS = 30000000L
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
  { //TO_CLOSE,
    .switch_pin = CLOSED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = HIGH,
    .engine = &engines[CLOSE_ENG]
  },
  { //TO_SEAL
    .switch_pin = SEALED_SWITCH_PIN,
    .switch_pol = LOW,
    .engine_dir_pol = HIGH,
    .engine = &engines[SEAL_ENG]
  },
  {
    .switch_pin = UNSEALED_SWITCH_PIN,
    .switch_pol = LOW,
    .engine_dir_pol = LOW,
    .engine = &engines[SEAL_ENG]
  }
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
  Movement* how_to_move;
};


struct Scen_Context {
  bool active;
  Prim_Context (*scenario_func)(Prim_Context, Movement);
  Scenario_State scenario_state;
  Scenario_Type scenario_type;
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
  Scen_Context* scen_context; //Указатель на контекст сценариев. Нужен чтобы инициализировать сценарий при нажатии на кнопку.
};



//TODO структура управления движением от концевки до его выключения



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
  if (previous_activation_time == 0) {
    previous_activation_time = micros();
  }

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

  if (previous_activation_time == 0) {
    previous_activation_time = micros();
  }

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
  if (reset) {
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




int movement_to_switch(Prim_Context prim_context, State state) {
  //Абстрактный примитив первого уровня «Движение мотора W со скоростью X в направлении Y до срабатывания концевого выключателя Z».
  //Возвращает код результата работы. 0 - авария, 1 - успех, 2 - работа не закончена.

  //Проверка таймаута
  if (prim_context.time_of_timeout - micros() < 0) {
    return 0;
  }

  //Проверка состояния концевика.
  //TODO вынести проверку концевиков в отдельную функцию.
  if (digitalRead(prim_context.how_to_move->switch_pin) == prim_context.how_to_move->switch_pol)
  {
    //Ардуино хранит цифровые сигналы не как булеву переменную, а как значения HIGH и LOW. Чему равны эти значения зависит от имплементации, логическое отрицание
    //записанного в переменную значения работать не будет. Но !digitalRead(pin) должно работать и инвертировать полученное значение.
    digitalWrite(prim_context.how_to_move->engine->enable_pin, LOW);
    return 1;
  }


  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.

  if (state.stop_state == true)
  {
    digitalWrite(prim_context.how_to_move->engine->enable_pin, LOW);
    return 0;
  }

  //Запись сигнала direction.
  digitalWrite(prim_context.how_to_move->engine->dir_pin, prim_context.how_to_move->engine_dir_pol);

  //Запись сигнала step обратного текущему. Данное заклинание должно работать по причине указанной выше.
  digitalWrite(prim_context.how_to_move->engine->step_pin, !digitalRead(prim_context.how_to_move->engine->step_pin));

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
  static int last_signal_time_close_sw = 2147483646;
  static const int delay_button; //задержка прочтения нажатия кнопки, в микросекундах
  static const int delay_switch;
  //функция обработки нажатых кнопок с учетом дребезга. для кнопок задержка +- 20 милисекунд, для концевиков может быть меньше, +-10. Возвращает активный сценарий.

  //если есть активный сценарий, то даже не смотреть на кнопки

  //TODO переписать на events, разобраться как учитывать время. Можно хранить время для каждого сигнала, но это звучит избыточно, возможно существует решение лучше.

  if (digitalRead(STOP_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_stop > delay_button) {
      last_signal_time_stop = 2147483646;
      state.stop_state = true;
    }
    else {
      last_signal_time_stop = micros();
    }
  }
  if (digitalRead(CLOSED_SWITCH_PIN) == HIGH) {
    if (micros() - last_signal_time_close_sw > delay_button) {
      last_signal_time_close_sw = 2147483646;
      state.close_switch_state = true;
    }
    else {
      last_signal_time_close_sw = micros();
    }

  }

  if (digitalRead(SEALED_SWITCH_PIN) == HIGH) {
    if (micros() - last_signal_time_seal_sw > delay_button) {
      last_signal_time_seal_sw = 2147483646;
      state.seal_switch_state = true;
    }
    else {
      last_signal_time_seal_sw = micros();
    }

  }


  if (digitalRead(UNSEALED_SWITCH_PIN) == HIGH) {
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

  if (digitalRead(CLOSE_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_close > delay_button) {
      last_signal_time_close = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = CLOSING;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_close = micros();
    }
  }

  if (digitalRead(SEAL_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_seal > delay_button) {
      last_signal_time_seal = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = SEALING;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_seal = micros();
    }
  }

  if (digitalRead(UNSEAL_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_unseal > delay_button) {
      last_signal_time_unseal = 2147483646;
      state.close_button_state = true;
      state.scen_context->scenario_type = UNSEALING;
      state.scen_context->active = true;
      return state;
    }
    else {
      last_signal_time_unseal = micros();
    }

  }
  return state;
}


Prim_Context init_primitive(Prim_Context prim_context, Movement* movement) {
  prim_context.how_to_move = movement;
  prim_context.current_inc_step = 16 * prim_context.how_to_move->engine->v_start;
  prim_context.inc_vel_end = 16 *  prim_context.how_to_move->engine->v_end;
  prim_context.current_inc_acc = (16 * (prim_context.inc_vel_end - prim_context.current_inc_step)) / ((prim_context.how_to_move->engine->v_end - prim_context.how_to_move->engine->v_start) / prim_context.how_to_move->engine->acc);
  prim_context.time_of_timeout = micros() + prim_context.how_to_move->engine->timeout_uS;

  //Запись сигнала enable.
  digitalWrite(prim_context.how_to_move->engine->enable_pin, prim_context.how_to_move->engine->enable_pol);

  return prim_context;
}



Scen_Context scenario_sealing(Prim_Context* prim_context, Scen_Context scen_context, State* state, Movement movements[3]) {
  int prim_result;
  switch (scen_context.scenario_state)
  {
    case (CHECK_CLOSED):
      if (state->close_switch_state) {
        scen_context.scenario_state = INIT_SEALED;
        return scen_context;
      }
      scen_context.scenario_state = INIT_UNSEALED;
      return scen_context;
    case (INIT_UNSEALED):
      *prim_context = init_primitive(*prim_context, &movements[2]);
      scen_context.scenario_state = WORK_SEALED;
      return scen_context;
    case (WORK_UNSEALED):
      if (prim_context->step_event) {
        prim_result = movement_to_switch(*prim_context, *state);
        prim_context->step_event = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = INIT_CLOSED;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (INIT_CLOSED):
      *prim_context = init_primitive(*prim_context, &movements[0]);
      scen_context.scenario_state = WORK_CLOSED;
      return scen_context;
    case (WORK_CLOSED):
      if (prim_context->step_event) {
        prim_result = movement_to_switch(*prim_context, *state);
        prim_context->step_event = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = INIT_SEALED;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (INIT_SEALED):
      *prim_context = init_primitive(*prim_context, &movements[1]);
      scen_context.scenario_state = WORK_SEALED;
      return scen_context;
    case (WORK_SEALED):
      if (prim_context->step_event) {
        prim_result = movement_to_switch(*prim_context, *state);
        prim_context->step_event = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (SCENARIO_EXIT):
      scen_context.scenario_state = LAST_SC_STATE;
      scen_context.scenario_type = LAST_SCENARIO;
      scen_context.active = false;
      return scen_context;
    case (LAST_SC_STATE):
      return scen_context;
    default:
      return scen_context;
  }
}

Scen_Context scenario_unsealing(Prim_Context* prim_context, Scen_Context scen_context, State* state, Movement movements[3]) {
  int prim_result;
  switch (scen_context.scenario_state)
  {
    case (CHECK_UNSEALED):
      if (state->unseal_switch_state) {
        scen_context.scenario_state = SCENARIO_EXIT;
        return scen_context;
      }
      scen_context.scenario_state = INIT_UNSEALED;
      return scen_context;
    case (INIT_UNSEALED):
      *prim_context = init_primitive(*prim_context, &movements[2]);
      scen_context.scenario_state = WORK_SEALED;
      return scen_context;
    case (WORK_UNSEALED):
      if (prim_context->step_event) {
        prim_result = movement_to_switch(*prim_context, *state);
        prim_context->step_event = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (SCENARIO_EXIT):
      scen_context.scenario_state = LAST_SC_STATE;
      scen_context.scenario_type = LAST_SCENARIO;
      scen_context.active = false;
      return scen_context;
    case (LAST_SC_STATE):
      return scen_context;
    default:
      return scen_context;
  }
}

Scen_Context scenario_closing(Prim_Context* prim_context, Scen_Context scen_context, State* state, Movement movements[3]) {
  int prim_result;
  switch (scen_context.scenario_state)
  {
    case (CHECK_CLOSED):
      if (state->close_switch_state) {
        scen_context.scenario_state = SCENARIO_EXIT;
        return scen_context;
      }
      scen_context.scenario_state = INIT_CLOSED;
      return scen_context;
    case (INIT_CLOSED):
      *prim_context = init_primitive(*prim_context, &movements[0]);
      scen_context.scenario_state = WORK_CLOSED;
      return scen_context;
    case (WORK_CLOSED):
      if (prim_context->step_event) {
        prim_result = movement_to_switch(*prim_context, *state);
        prim_context->step_event = false;
      }
      switch (prim_result)
      {
        case 0: //отработка аварии выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 1: //отработка успеха выполнения примитива
          *prim_context = vel_accel(*prim_context, true);
          state->stop_state = false;
          scen_context.scenario_state = SCENARIO_EXIT;
          return scen_context;
        case 2:
          return scen_context;
        default:
          return scen_context;
      }
    case (SCENARIO_EXIT):
      scen_context.scenario_state = LAST_SC_STATE;
      scen_context.scenario_type = LAST_SCENARIO;
      scen_context.active = false;
      return scen_context;
    case (LAST_SC_STATE):
      return scen_context;
    default:
      return scen_context;
  }
}

Prim_Context prim_context;

Scen_Context scen_context;


State state;


Events events;




void setup() {

  scen_context.scenario_type = LAST_SCENARIO;
  scen_context.scenario_state = LAST_SC_STATE;
  state.scen_context = &scen_context;

  Serial.begin(9600);
  //define pin modes
  

}







//Инкремент аккумулятора шага. Изначально равен начальной скорости.
//int accumulator_inc_step = Engine.v_start;
void loop() {

  //обзвон кнопок и свичей. buttons_processing инициализирует сценарий при записи состояния кнопки в state
  state = buttons_processing(state, events, scen_context.active);

  //TODO добавить функцию приведения состояния лампочек в соответствие state

  prim_context = vel_accel(prim_context);

  switch (scen_context.scenario_type)
  {
    case (CLOSING):
      scen_context = scenario_closing(&prim_context, scen_context, &state, movements);
      break;
    case (SEALING):
      scen_context = scenario_sealing(&prim_context, scen_context, &state, movements);
      break;
    case (UNSEALING):
      scen_context = scenario_unsealing(&prim_context, scen_context, &state, movements);
      break;
    case (LAST_SCENARIO):
      break;
  }


}

