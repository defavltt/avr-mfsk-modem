#define delayfix 64 //timer 0 compensation

#define N 256
#define IX_LEN 4

uint8_t samples[N];
volatile uint16_t samplePos = 0;

float spectrum[IX_LEN];

float cos_t[IX_LEN];
float sin_t[IX_LEN];

const uint16_t window = 256; //buffer size
const float samplerate = 9615.0;

const float pi = 3.14159265359;

byte sine64[] = {140,153,165,177,188,199,209,218,226,234,240,246,250,253,255,255,255,253,250,246,240,234,226,218,209,199,188,176,165,152,140,128,115,103,90,79,67,56,46,37,29,21,15,9,5,2,0,0,0,2,5,9,15,21,29,37,46,56,67,78,90,102,115,128};
byte wave1out;
float wave1ptr;

//float modemtones[IX_LEN] = {1000, 1100, 1200, 1300};
float modemtones[IX_LEN];

float frequency = 1000.0f;
uint8_t modemspeed;
int modemtonespace = 100;
int modembasefreq = 1000;
uint8_t setmodemspeed = 15; //15,25,35,45
uint16_t modempreamble = 500; //0-1000
float modemthreshold = 3000; //set above noise level


uint16_t doublebitpointer;
bool modemtransmitting;
uint8_t modemsymbol; //being a local var instead of a global causes problems

volatile uint16_t tickcount;
volatile bool tick;

uint8_t statusled; //0 - off, 1 - on, 2 - blink
bool blinkled;
uint8_t blinktimer;

bool lastbuttonstate;
bool modetoggle; //0 - receiving, 1 - config
bool rtrans = 0;

uint16_t rtranstimer;

uint8_t header[2] = {'@', '!'}; //ballast of 2 bytes required after that a special character
uint8_t datamsg[34];
uint8_t datarecv[34];
uint8_t rxstatereturn;

void setup() {
  cli();
  initADC();
  sei();

  pinMode(2, OUTPUT); //led
  digitalWrite(2, HIGH);
  
  pinMode(6, OUTPUT); //pwm out

  pinMode(15, INPUT); //ptt, low to transmit

  pinMode(5, OUTPUT); //debug

  pinMode(4, INPUT_PULLUP); //button 2

  if(!digitalRead(4)) { //choose serial speed a startup
    while(!digitalRead(4)) { } //wait to let go of button
    Serial.begin(9600);
  } else {
    Serial.begin(115200);
  }

  analogReference(INTERNAL);

  //timer0
  TCCR0A = 0b10100011; //fast pwm mode
  TCCR0B = 0b00000001;

  //timer2
  TCCR2A = 0b00000010; //ctc mode
  TCCR2B = 0b00000111;
  OCR2A = 127;

  TIMSK2 = 0b00000010; //enable timer interrupts for sampling

  settones(modemtonespace, modembasefreq);
}

void loop() {
  bool buttonstate = digitalRead(4);
  if(buttonstate == LOW && buttonstate != lastbuttonstate) {
    modetoggle = !modetoggle;
    flashled(); //flash led to confirm press
  }
  lastbuttonstate = buttonstate;
  
  if(0 < Serial.available()) {
    if(modetoggle) { //config
      uint16_t value = Serial.parseInt();
      rtrans = doublebitread(value, 0); //rtrans
      modembasefreq = 375 * (doublebitread(value, 1) + 1); //base freq
      modemtonespace = 50 * (doublebitread(value, 2) + 1); //freq spacing
      setmodemspeed = 10.0 * (doublebitread(value, 3) + 1.5); //modem speed
      settones(modemtonespace, modembasefreq);
      rtranstimer = 0;
    } else { //receiving
      String serialread = Serial.readString();
      strcpy((char*)datamsg, serialread.c_str()); //put message from serial to buffer
      addheader(datamsg, sizeof(datamsg));
      modemtx(datamsg, strlen((char*)datamsg)+1, setmodemspeed, modempreamble);
      statusled = 2; //blink
    }
  }
  if(modetoggle) {
    detectclipping();
  } else {

    
    switch(statusled) {
      case 0: //off
        digitalWrite(2, 0);
        break;
      case 1: //on
        digitalWrite(2, 1);
        break;
      case 2: //blink
        blinktimer++;
        if(blinktimer > 16) {
          blinkled = !blinkled; //toggle
          digitalWrite(2, blinkled);
          blinktimer = 0;
        }
        break;
    }

    if(rtrans) {
      rtranstimer++;
      if(rtranstimer > 1023) {
        rtranstimer = 0; //reset
        modemtx(datamsg, strlen((char*)datamsg)+1, setmodemspeed, modempreamble);
      }
    }
    
    modemrx(datarecv, sizeof(datarecv), setmodemspeed, modemthreshold);
    statusled = 2; //blink
    if(rxstatereturn > 0) {
      bool validmsg = detectcorrectarrayshift(datarecv, sizeof(datarecv), 8, 2, '!'); //find right array double bit aligment
      if(validmsg) {
        removefirstchar(datarecv, sizeof(datarecv));
        Serial.println((char*)datarecv);
//        delay(500 * delayfix);
//        delay(500 * delayfix);
//        delay(500 * delayfix);
//        delay(500 * delayfix);
      }
    }
//    switch(rxstatereturn) {
//      case 1: //signal end
//        Serial.println("signal end");
//        break;
//      case 2: //array ovf
//        Serial.println("array ovf");
//        break;
//    }
  }
}

ISR(ADC_vect) {
//  PORTD = 0b10000000; //sampling rate debug
//  PORTD = 0b00000000;
  if(modemtransmitting) {
    wave1ptr += 64.0f * frequency / samplerate;
    while(wave1ptr >= 64.0f) {
      wave1ptr -= 64.0f;
    }
    OCR0A = (sine64[(int)wave1ptr]);
  } else {
    uint16_t sample = ADC;

    samples[samplePos++] = sample;
    
    if(samplePos >= N) {
      ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
    }
  }
}

ISR(TIMER2_COMPA_vect) {
  tickcount++;
  if(tickcount > modemspeed) {
    tickcount = 0; //reset
    tick = 1; //set tick
  }
}

void initADC() {
  ADMUX = 0b01000000;
  ADCSRA = 0b11101111;
  ADCSRB = 0; // Free-run mode
}

void goertzel(uint8_t *samples, float *spectrum) {
  float v_0, v_1, v_2;
  float re, im, amp;
    
  for (uint8_t k = 0; k < IX_LEN; k++) {
    float c = cos_t[k];
    float s = sin_t[k];
    
    float a = 2. * c;
    v_0 = v_1 = v_2 = 0;  
    for (uint16_t i = 0; i < N; i++) {
      v_0 = v_1;
      v_1 = v_2;
      v_2 = (float)(samples[i]) + a * v_1 - v_0;
    }
    re = c * v_2 - v_1;
    im = s * v_2;
    amp = sqrt(re * re + im * im);
    spectrum[k] = amp;        
  } 
}

float calcfftfreqvalue(float freq, float samplerate, uint16_t window) {
  float k = 0.5 + ((window * freq) / samplerate);
  return (2 * (pi / window)) * k;
}

uint8_t spectrum2binary(float threshold) {
//    while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish
  goertzel(samples, spectrum); //calculate fft each time the function is called
  debugpulse();
    samplePos = 0;
    ADCSRA |= _BV(ADIE); // Resume sampling interrupt
  for(byte i = 0; i < IX_LEN; i++) {
    if(spectrum[i] > threshold) {
      return i; //return active tone number (0-3) 4 - no tone
    }
  }
}

void doublebitbytewritemanual(uint8_t *input, uint8_t pointer, uint8_t value, uint8_t select) {
  input[pointer] |= value << select*2;
}

void doublebitbytewrite(uint8_t *input, uint8_t value) {
  uint16_t arrayptr = doublebitpointer / 4;
  input[arrayptr] |= value << (doublebitpointer % 4)*2;
  doublebitpointer++;
}

uint8_t doublebitread(uint8_t value, uint8_t select) {
  return (value >> select*2) & B00000011;
}

void modemtx(uint8_t *data, size_t len, uint16_t speed, uint16_t preamble) {
  const uint8_t freqoffset = 17; //compensate for fft offset
  uint8_t i = 0;
  uint8_t k = 0;
  modemspeed = speed;
  modemtransmitting = 1;
  digitalWrite(2, 1); //led on
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW); //ptt on
  frequency = modemtones[3] + freqoffset; //preamble
  asm("NOP"); //bodge
  delay(preamble * delayfix);
  debugpulse();
  tickcount = 0; //start immediately
  while(true) { //payload
    if(tick) { //timing based on adc sample rate
      frequency = modemtones[doublebitread(datamsg[k], i)]  + freqoffset;
      debugpulse();
      i++;
      if(i >= 4) { //for each double bit (symbol)
        i = 0;
        k++;
        if(k >= len) { //for each byte from array
          pinMode(15, INPUT); //ptt off
          modemtransmitting = 0; //stop signal
          tick = 0;
          return; //exit while loop
        }
      }
      tick = 0; //clear tick
    }
  }
}

void modemrx(uint8_t *data, size_t len, uint16_t speed, float threshold) {
  modemspeed = speed;
  doublebitpointer = 0; //reset
  uint16_t symbolcount = 0;
  modemtransmitting = 0;
  if(spectrum2binary(threshold) == 4) { 
    rxstatereturn = 0;
    return;
  }
  if(spectrum2binary(threshold) == 3) { //detect preamble
//    debugpulse();
    digitalWrite(2, 1); //led on
    erasedata(data, len); //erase message buffer if a preamble is detected
    while(spectrum2binary(threshold) == 3) { //wait for preamble to end
      //pause
    }
  }
  delay(132 * delayfix); //132 - speed 15 / 265 - speed 30
  tickcount = 0; //start immediately
  while(true) { //start assembling data
    if(tick) { //timing based on adc sample rate
      modemsymbol = spectrum2binary(threshold); //detect tones
//      Serial.println(modemsymbol);
      if(modemsymbol == 4) { //no tone, stop receiving
        rxstatereturn = 1;
        return;
      }
//      debugpulse();
      doublebitbytewrite(data, modemsymbol); //put symbols into bytes, into an array
      symbolcount++;
      if(symbolcount / 4 > len) { //stop receiving to prevent overflow
        tick = 0; //clear tick
        rxstatereturn = 2;
        return;
      }
      tick = 0; //clear tick
    }
  }
}

void debugpulse() {
  digitalWrite(7, HIGH);
  delay(1);
  digitalWrite(7, LOW);
}

void detectclipping() {
  digitalWrite(2, LOW);
  for(byte i = 0; i < 8; i++) {
    if(samples[i] <= 10 || samples[i] >= 240) {
      digitalWrite(2, HIGH);
    }
    samplePos = 0;
    ADCSRA |= _BV(ADIE); // Resume sampling interrupt
  }
}

void doublebitshift(uint8_t *input, size_t len, bool direction) { //0 - left, 1 - right
  uint8_t bitbuf = 0;
  if(direction) { //right
    for(byte i = len; i > 0; i--) {
      input[i-1] >>= 2;
      bitbuf = doublebitread(input[i-2], 0);
      doublebitbytewritemanual(input, i-1, bitbuf, 3);
    }
  } else { //left
    for(byte i = 0; i < len; i++) {
      input[i] <<= 2;
      bitbuf = doublebitread(input[i+1], 3);
      doublebitbytewritemanual(input, i, bitbuf, 0);
    }
  }
}

bool detectcorrectarrayshift(uint8_t *input, size_t len, uint8_t level, uint8_t depth, uint8_t character) {
  for(byte i = 0; i < level; i++) { //sift to the left
    doublebitshift(input, len, 0);
//    Serial.print('<'); Serial.println(i);
    for(byte k = 0; k < depth; k++) { //search from the array beginning at a desired length
      if(input[k] == character) {
        return 1; //return valid
      }
    }
  }
  return 0; //invalid message
}

void calcfftvalues(float *input) {
  for(byte i = 0; i < IX_LEN; i++) { //calculate values for fft to work with
    float w = calcfftfreqvalue(modemtones[i], samplerate, window);
    cos_t[i] = cos(w);
    sin_t[i] = sin(w);
  }
}

void settones(int space, int freq) {
  for(byte i = 0; i < IX_LEN; i++) {
    modemtones[i] = freq + (space * i);
  }
  calcfftvalues(modemtones);
}

void erasedata(uint8_t *data, size_t len) {
  for(byte i = 0; i < len; i++) {
    data[i] = 0;
  }
}

void flashled() {
  for(byte i = 0; i < 4; i++) {
    digitalWrite(2, 1);
    delay(100 * delayfix);
    digitalWrite(2, 0);
    delay(100 * delayfix);
  }
}

void removefirstchar(uint8_t *data, size_t len) {
  uint8_t buf = 0;
  for(byte i = 0; i < len; i++) {
    buf = data[i+1];
    data[i] = buf;
  }
}

void movecharstoright(uint8_t *data, size_t len) {
  uint8_t buf = 0;
  for(byte i = len-1; i > 0; i--) { //might be len-1
    buf = data[i-1];
    data[i] = buf;
  }
}

void addheader(uint8_t *data, size_t len) {
  movecharstoright(data, len); //make 2 byte free space in the beginning for the header
  movecharstoright(data, len);
  data[0] = header[0];
  data[1] = header[1];
}
