
#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder  myenc;
unsigned long encoderLastToggled;
bool          encoderPaused = false;

const uint8_t PIN_ROTARY_A = 26;
const uint8_t PIN_ROTARY_B = 27;
const uint8_t PIN_ROTARY_Z = 32;
const uint8_t PIN_PWM      = 25;
const uint8_t PIN_DIR      = 33;
const uint8_t PWM_CHANNEL  = 0;

const double INTEGRAL_MAX = 1000.;
const double INTEGRAL_MIN = -1000.;

const uint16_t FREQUENCY       = 20000;
const uint8_t  RESOLUTION_BITS = 8;

// 制御周期
const uint16_t CONTROL_CYCLE = 2000;

// 目標位置
const int16_t angle  = 200;
int           target = map(angle, 0, 360, 0, 4096);

const double KP = 3.;
const double KI = 0.01;
const double KD = 0.01;

volatile long pos        = 0;
double        integral   = 0;
double        prev_error = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
    Serial.begin(115200); // put function declarations here:

    ESP32Encoder::useInternalWeakPullResistors = puType::none;
    myenc.attachHalfQuad(PIN_ROTARY_A, PIN_ROTARY_B);

    myenc.clearCount();

    pinMode(PIN_ROTARY_A, INPUT);
    pinMode(PIN_ROTARY_B, INPUT);
    pinMode(PIN_ROTARY_Z, INPUT);
    pinMode(PIN_DIR, OUTPUT);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
    ledcSetup(PWM_CHANNEL, FREQUENCY, RESOLUTION_BITS);

    // 初期位置合わせ
    bool prevZ = gpio_get_level((gpio_num_t)PIN_ROTARY_Z);

    ledcWrite(PWM_CHANNEL, 30); // 低速が安全

    while (1) {
        bool nowZ = gpio_get_level((gpio_num_t)PIN_ROTARY_Z);

        if (!prevZ && nowZ) { // 立ち上がり検出
            myenc.clearCount();
            break;
        }

        prevZ = nowZ;
    }
    ledcWrite(PWM_CHANNEL, 0);
    Serial.println("start");
}

void loop() {
    static unsigned long prev_time = micros();
    unsigned long        now_time  = micros();
    if (now_time - prev_time >= CONTROL_CYCLE) {
        prev_time += CONTROL_CYCLE;
        double dt = CONTROL_CYCLE / 1e6;

        /* NOTE: 割り込みが発生したときにposの値は違う値になってしまうので、
        別の変数にコピーして使用する*/
        long pos_now = 0;
        pos_now      = myenc.getCount();

        int error = target - pos_now;
        // ±2048の範囲に収める
        if (error > 2048) error -= 4096;
        if (error < -2048) error += 4096;

        double derivative = (error - prev_error) / dt;
        prev_error        = error;

        integral += error * dt;
        // I項によるオーバーフローを防ぐため
        integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double control = KP * error + KI * integral + KD * derivative;

        uint8_t pwm = constrain(abs(control), 0., 255.);

        digitalWrite(PIN_DIR, control > 0 ? HIGH : LOW);
        ledcWrite(PWM_CHANNEL, pwm);

        long display_pos = pos_now % 4096;
        if (display_pos < 0) display_pos += 4096;
        Serial.print("pos  ");
        Serial.printf("%d %d\n", display_pos, target);
    }
}