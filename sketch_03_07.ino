//define pins

#define buffer_length (24)

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
enum Movements_Item { TO_CLOSE, TO_SEAL, TO_UNSEAL, TO_UNCLOSE, FROM_SEAL, FROM_UNSEAL, MOVEMENTS_NUMBER};

enum Scenario_Type {NONE, CLOSING, SEALING, UNSEALING, LAST_SCENARIO};

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
  {//TO_UNSEAL, 2
    .switch_pin = UNSEALED_SWITCH_PIN,
    .switch_pol = LOW,
    .engine_dir_pol = LOW,
    .engine = &engines[SEAL_ENG]
  },
  {//TO_UNCLOSE, 3
    .switch_pin = OPENED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = LOW,
    .engine = &engines[CLOSE_ENG]
  },
  {//FROM_SEAL, 4
    .switch_pin = SEALED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = LOW,
    .engine = &engines[SEAL_ENG]
  },
  {//FROM_UNSEAL, 5
    .switch_pin = UNSEALED_SWITCH_PIN,
    .switch_pol = HIGH,
    .engine_dir_pol = HIGH,
    .engine = &engines[SEAL_ENG]
  },
};

typedef struct{
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
  bool close_button_state;
  bool seal_switch_state;
  bool seal_button_state;
  bool unseal_switch_state;
  bool unseal_button_state; //Указатель на контекст сценариев. Нужен чтобы инициализировать сценарий при нажатии на кнопку.
} State;





typedef struct  {
  bool button_close_event;
  bool button_unseal_event;
  bool button_seal_event;
  bool button_stop_event;
  bool switch_stop_event;
  bool switch_seal_event;
  bool switch_unseal_event;
} Events;


void vel_accel (Prim_Context* prim_context); //Функция регулирующая запуск аккумуляторов. Генерирует step_event
int movement_to_switch(Prim_Context* prim_context, State state); //Абстрактный примитив движения в сторону выключателя. Осуществляет движение мотором
void init_primitive(Prim_Context* prim_context, Movement* movement); //Функция инициализирующая примитив и аккумуляторы. 
void stop_primitive(Prim_Context* prim_context);

void init_primitive(Prim_Context* prim_context, Movement* movement) {

  double multiplier = (2l<<buffer_length)/1e6;
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

void stop_primitive(Prim_Context* prim_context){
  if(prim_context->how_to_move->engine->enable_pol == HIGH)
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
  if(!prim_context->active) return;
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
  //TODO вынести проверку концевиков в отдельную функцию.
  if (digitalRead(prim_context->how_to_move->switch_pin) == prim_context->how_to_move->switch_pol)
  {
    prim_context->prim_state = SUCCESS;
    stop_primitive(prim_context);
    return;
  }


  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.
  if (state->stop_state == true)
  {
    prim_context->prim_state = STOPPED;
    stop_primitive(prim_context);
    return;
  }

  if(prim_context->prim_state == WORKING){
    if (prim_context->step_event) {
        prim_context->step_event = false;
        //Запись сигнала step обратного текущему.
        if(digitalRead(prim_context->how_to_move->engine->step_pin) == HIGH)
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

//перенести назначение и инициализацию сценариев в диспетчер сценариев!!!!!
void buttons_processing(State* state, Events* events, bool scenario_active) {
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
      state->stop_state = true;
    }
    else {
      last_signal_time_stop = micros();
    }
  }
  if (digitalRead(CLOSED_SWITCH_PIN) == HIGH) {
    if (micros() - last_signal_time_close_sw > delay_button) {
      last_signal_time_close_sw = 2147483646;
      state->close_switch_state = true;
    }
    else {
      last_signal_time_close_sw = micros();
    }

  }

  if (digitalRead(SEALED_SWITCH_PIN) == HIGH) {
    if (micros() - last_signal_time_seal_sw > delay_button) {
      last_signal_time_seal_sw = 2147483646;
      state->seal_switch_state = true;
    }
    else {
      last_signal_time_seal_sw = micros();
    }

  }


  if (digitalRead(UNSEALED_SWITCH_PIN) == HIGH) {
    if (micros() - last_signal_time_unseal_sw > delay_button) {
      last_signal_time_unseal_sw = 2147483646;
      state->unseal_switch_state = true;
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
      state->close_button_state = true;
      return state;
    }
    else {
      last_signal_time_close = micros();
    }
  }

  if (digitalRead(SEAL_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_seal > delay_button) {
      last_signal_time_seal = 2147483646;
      state->close_button_state = true;
      return state;
    }
    else {
      last_signal_time_seal = micros();
    }
  }

  if (digitalRead(UNSEAL_BUTTON_PIN) == HIGH) {
    if (micros() - last_signal_time_unseal > delay_button) {
      last_signal_time_unseal = 2147483646;
      state->close_button_state = true;
      return state;
    }
    else {
      last_signal_time_unseal = micros();
    }

  }
  return state;
}

//TODO добавить движение от концевиков, привести сценарии в человеческий вид.

void scenario_sealing(Prim_Context* prim_context, Scen_Context* scen_context, State* state, Movement movements[3]) {
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
    case (INIT_UNCLOSE):
      {
      init_primitive(prim_context, &movements[3]);
      scen_context->scenario_state = WORK_UNCLOSE;
      break;
      } 
    case (WORK_UNCLOSE):
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
    case (INIT_FROM_SEAL):
      init_primitive(prim_context, &movements[4]);
      scen_context->scenario_state = WORK_FROM_SEAL;
      break;
    case (WORK_FROM_SEAL):
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
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}

void scenario_unsealing(Prim_Context* prim_context, Scen_Context* scen_context, State* state, Movement movements[3]) {
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
      break;
    case (LAST_SC_STATE):
      break;
    default:
      break;
  }
}
void scenario_closing(Prim_Context* prim_context, Scen_Context* scen_context, State* state, Movement movements[3]) {

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
    case (SCENARIO_EXIT):
      scen_context->scenario_state = LAST_SC_STATE;
      scen_context->scenario_type = LAST_SCENARIO;
      scen_context->active = false;
      state->close_button_state = false;
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

  scen_context.scenario_type = LAST_SCENARIO;
  scen_context.scenario_state = LAST_SC_STATE;

  Serial.begin(9600);
  //define pin modes

  //При запуске вернуть двигатель закрытия в начальное положение

  

}






//Инкремент аккумулятора шага. Изначально равен начальной скорости.
//int accumulator_inc_step = Engine.v_start;
void loop() {

  //обзвон кнопок и свичей. buttons_processing инициализирует сценарий при записи состояния кнопки в state
  buttons_processing(&state, &events, scen_context.active);

  //TODO добавить функцию приведения состояния лампочек в соответствие state

  vel_accel(&prim_context);

  movement_to_switch(&prim_context, &state);

  if (scen_context.active) {
    switch (scen_context.scenario_type)
    {
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
    if(state.close_button_state){
      scen_context.scenario_type = CLOSING;
      scen_context.active = true;
    }else if (state.seal_button_state){
      scen_context.scenario_type = SEALING;
      scen_context.active = true;
    } else if (state.unseal_button_state){
      scen_context.scenario_type = UNSEALING;
      scen_context.active = true;
    }
  }
}