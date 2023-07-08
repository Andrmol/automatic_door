//define pins


enum switch_state {closed, sealed, unsealed};

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

bool phase_accumulator_step (int accumulator_inc_step, bool reset = false) {
  //Фазовый аккумулятор регулирующий ход мотора. Начальный инкремент соответствует начальной скорости, увеличивается аккумулятором ускорения.
  static long accumulator_step;
  static const int buffer_length_step = 32;
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

  const int accumulator_inc_acc;
  static long accumulator_acc;
  static const int buffer_length_acc = 32;
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

int movement_to_switch(Movement movement, bool stop_state) {
  //Абстрактный примитив первого уровня «Движение мотора W со скоростью X в направлении Y до срабатывания концевого выключателя Z».
  //Возвращает код результата работы. 0 - авария, 1 - успех, 2 - работа не закончена.
  static int time_of_start;

  //Запуск счетчика общего таймаута.
  if (time_of_start != 0)
  {
    time_of_start = micros();
    //Запись сигнала enable. Таким образом будет вызываться только при первом запуске функции. Предположительно существует более красивое решение.
    digitalWrite(movement.engine.enable_pin, movement.engine.enable_pol);
  }

  //Проверка состояния концевика.
  if (digitalRead(movement.switch_pin) == movement.switch_pol)
  {
    time_of_start = 0;
    //Ардуино хранит цифровые сигналы не как булеву переменную, а как значения HIGH и LOW. Чему равны эти значения зависит от имплементации, логическое отрицание
    //записанного в переменную значения работать не будет. Но !digitalRead(pin) должно работать и инвертировать полученное значение.
    digitalWrite(movement.engine.enable_pin, LOW);
    return 1;
  }


  //Проверка аварийного стопа. Сброс состояния аварийного стопа происходит на выходе из функции.
  if (stop_state == true)
  {
    time_of_start = 0;
    digitalWrite(movement.engine.enable_pin, LOW);
    return 0;
  }

  //Запись сигнала direction.
  digitalWrite(movement.engine.dir_pin, movement.engine_dir_pol);

  //Запись сигнала step обратного текущему. Данное заклинание должно работать по причине указанной выше.
  digitalWrite(movement.engine.step_pin, !digitalRead(movement.engine.step_pin));

  //Проверка таймаута.
  if (micros() - time_of_start < movement.engine.timeout)
  {
    time_of_start = 0;
    digitalWrite(movement.engine.enable_pin, LOW);
    return 0;
  }

  return 2;
}

int inputs_processing(State state, int active_scenario) {
  static int last_signal_time;
  static const int delay_button;
  static const int delay_switch;
  //функция обработки нажатых кнопок с учетом дребезга. для кнопок задержка +- 20 милисекунд, для концевиков может быть меньше, +-10

  if (digitalRead(1) == HIGH) {
    
  }

  return active_scenario;
}

int scenario_close (Movement movement) {
  if (accumulator_inc_step < Engine.v_end) {
    if (phase_accumulator_acc() == true) {
      accumulator_inc_step = accumulator_inc_step * Engine.acc;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, запускает примитив при возвращении true
  if (phase_accumulator_step(accumulator_inc_step) == true) {
    //Запуск примитива.
    process_state = movement_to_switch(movement, state.stop_state);
    if (process_state == 0)
    {
      //При получении из примитива состояний "успех" или "провал" сбрасывает состояния релевантных кнопок. Сбрасывает аккумуляторы.
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
    if (process_state == 1)
    {
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
  }
}
}

int scenario_seal (Movement movement) {
  if (accumulator_inc_step < Engine.v_end) {
    if (phase_accumulator_acc() == true) {
      accumulator_inc_step = accumulator_inc_step * Engine.acc;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, запускает примитив при возвращении true
  if (phase_accumulator_step(accumulator_inc_step) == true) {
    //Запуск примитива.
    process_state = movement_to_switch(movement, state.stop_state);
    if (process_state == 0)
    {
      //При получении из примитива состояний "успех" или "провал" сбрасывает состояния релевантных кнопок. Сбрасывает аккумуляторы.
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
    if (process_state == 1)
    {
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
  }
}
}

int scenario_unseal (Movement movement) {
  if (accumulator_inc_step < Engine.v_end) {
    if (phase_accumulator_acc() == true) {
      accumulator_inc_step = accumulator_inc_step * Engine.acc;
    }
  }
  //проверка аккумулятора хода. Запускается всегда, запускает примитив при возвращении true
  if (phase_accumulator_step(accumulator_inc_step) == true) {
    //Запуск примитива.
    process_state = movement_to_switch(movement, state.stop_state);
    if (process_state == 0)
    {
      //При получении из примитива состояний "успех" или "провал" сбрасывает состояния релевантных кнопок. Сбрасывает аккумуляторы.
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
    if (process_state == 1)
    {
      stop_state = false;
      button1_state = false;
      accumulator_inc_step = Engine.v_start;
      phase_accumulator_acc(reset = true);
      phase_accumulator_step(reset = true);
    }
  }
}
}

int scenario_manager(State state, int active_scenario, Movement movement) {
  //функция запуская и контроля статусов выполнения сценариев.
  static int active_scenario;

  switch (active_scenario):
    case closed:
    scenario_close();
  case unsealed:
    scenario_unsealed();
  case sealed:
    scenario_seal ();
  }


  void setup() {

  Serial.begin(9600);
  //define pin modes

}


State state;
int active_scenario;
int process_state;

//Инкремент аккумулятора шага. Изначально равен начальной скорости.
int accumulator_inc_step = Engine.v_start;
void loop() {

  active_scenario = inputs_processing(state);

  scenario_manager(state, active_scenario);


  if (button1_state == true) {
    {
      //проверка аккумулятора ускорения. Запускается если скорость не превышает максимальной.
      if (accumulator_inc_step < Engine.v_end) {
        if (phase_accumulator_acc() == true) {
          accumulator_inc_step = accumulator_inc_step * Engine.acc;
        }
      }
      //проверка аккумулятора хода. Запускается всегда, запускает примитив при возвращении true
      if (phase_accumulator_step(accumulator_inc_step) == true) {
        //Запуск примитива.
        process_state = movement_to_switch(movement, state.stop_state);
        if (process_state == 0)
        {
          //При получении из примитива состояний "успех" или "провал" сбрасывает состояния релевантных кнопок. Сбрасывает аккумуляторы.
          stop_state = false;
          button1_state = false;
          accumulator_inc_step = Engine.v_start;
          phase_accumulator_acc(reset = true);
          phase_accumulator_step(reset = true);
        }
        if (process_state == 1)
        {
          stop_state = false;
          button1_state = false;
          accumulator_inc_step = Engine.v_start;
          phase_accumulator_acc(reset = true);
          phase_accumulator_step(reset = true);
        }
      }
    }

  }
}

}
