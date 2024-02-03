#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include "commands.h"
#include <Servo.h>

////////////// DISTANCE SENSOR Variables   ////////
VL53L0X tof;
float distance, prev_distance;

////////////// COLOR SENSOR Variables  ////////
double RED,GREEN,BLUE;
commands_t serial_commands;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

/////////////// State Machines ////////////////

typedef struct {
  bool dir;  //direction 
  double pos;
} motors;


typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

motors MB, MR, MH, MC; // MB - Motor Base; MR - Motor Reach; MH - Motor Height; MC - Motor Catch
fsm_t fsm;

///////////////// Update States ///////////////

void update_state(fsm_t & fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis resets
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

////////////// Set Servos ////////
Servo M_base,M_reach,M_height,M_catch;


///////////// Global Use Variables ////////////////////
unsigned long interval;
unsigned long currentMillis, previousMillis;
int8_t loop_count = 0, offset = 0;
double obj_pos = 0, dist_to_reach = 0;    
bool start_activity, in_reach = false, in_height = false, in_sight = false, got_color = false, in_place = false;
bool colors[4] = {false,false,false,false}; // este array guarda nas posições 0 a 3 a cor que está a ser vista no sensor na seguinte ordem: RED_GREEN_BLUE_YELLOW

void calc_next_state(){
  if (start_activity == false){
    fsm.new_state = 0;
    loop_count = 0;
    dist_to_reach = 0;

  } else if (fsm.state == 0 && start_activity == true){
    fsm.new_state = 1;

  } else if (fsm.state == 1 && distance <= 0.185 && distance  >= 0.088 && abs(distance - prev_distance) < 0.02){ //o outro código que estava a usar antes: (mean_dist < 0.040 && mean_dist > 0.010)
    dist_to_reach += distance;
    obj_pos += MB.pos;
    loop_count ++;
    in_sight = true;

  } else if (fsm.state == 1 && (distance > 0.185 || distance  < 0.088 || abs(distance - prev_distance) > 0.05 ) && in_sight == true){
    //if (MB.dir == true) offset = -3;      // ajustes de offset
    if (MB.dir == false) offset = -5;

    in_sight = false;
    dist_to_reach = dist_to_reach / (loop_count);      
    obj_pos = int(obj_pos / (loop_count)) + offset;
    fsm.new_state = 2;

    if (dist_to_reach < 0.15) dist_to_reach = dist_to_reach - 0.015; // ajuste para baixas distancias
    if (dist_to_reach >= 0.15) dist_to_reach = dist_to_reach - 0.015;

  }  else if (fsm.state == 2 && in_reach == true && in_height == true){ 
    dist_to_reach = 0;
    obj_pos = 0;
    loop_count = 0;
    fsm.new_state = 3;
    in_height = false;
    in_reach = false;

  } else if (fsm.state == 3 && MC.pos == 100){
    fsm.new_state = 4; // estado 4 é para recolher braço

  } else if (fsm.state == 4 && MH.pos == 120 && MR.pos == 60){
    fsm.new_state = 5;                      // ir para a posição do sensor de cor
   
  } else if (fsm.state == 5 && MB.pos == 27){ 
    fsm.new_state = 6;
    
  } else if (fsm.state == 6 && got_color == true && MR.pos == 100 && MH.pos == 65){
    fsm.new_state = 7;
    got_color = false;

  } else if (fsm.state == 7 && in_place == true){
    fsm.new_state = 8;
    in_place = false;

  } else if (fsm.state == 8 && MR.pos == 60 && MH.pos == 120 && MB.pos == 50){
    fsm.new_state = 1;

  } else if (fsm.state == 1 && fsm.tis >= 25000){
    fsm.new_state = 9;
  }
}

void calc_outputs(){

  if (fsm.state == 0){ //estado em que o braço inicia // posições standard de height e reach
    MR.pos = 60;
    MB.pos = 180;
    MC.pos = 0;
    MH.pos = 120;
    
    M_base.attach(0, 544, 2344); 
    M_reach.attach(2, 544, 2344); 
    M_catch.attach(4, 544, 2344); 
    M_height.attach(6, 544, 2344); 

  } else if (fsm.state == 1){ //estado em que o braço está a fazer scan para encontrar peças
    if (MB.pos <= 50){
      MB.dir = true;
    } else if(MB.pos >= 180){
      MB.dir = false;
    }
    
    if (MB.dir == true){
      MB.pos ++;
    } else if (MB.dir == false){
      MB.pos --;
    }

  } else if (fsm.state == 2){ //estado em que o braço busca a peça (base fixa, altura e reach aumentam aos poucos para não se fazer movimentos bruscos e alterar a posição da peça)
    
    double alpha = 590 * dist_to_reach + 54;
    double beta = 1.3 * alpha - 112;

    if (MR.pos < alpha){
      MR.pos ++;
    } else{
      in_reach = true;
    } 
    
    if (MH.pos > beta){
      MH.pos = MH.pos - 3;
    } else {
      in_height = true;
    }

    if (MB.pos < obj_pos){
      MB.pos ++;
    } else if (MB.pos > obj_pos){
      MB.pos --;
    }
    

  } else if (fsm.state == 3){ //depois do reach atingir a posição definida, temos que fechar a garra 
    MC.pos = MC.pos + 5;

  } else if (fsm.state == 4){ //depois da garra fechar temos que levantar o braço
    MH.pos = MH.pos + 3;
    MR.pos = MR.pos - 3;
    
    if (MH.pos >= 120){
      MH.pos = 120;
    } 
    if (MR.pos <= 60){
      MR.pos = 60;
    }

  } else if (fsm.state == 5){ //levar a peça para a posição de ver a cor
    MB.pos = MB.pos - 2;

    if (MB.pos <= 27){
      MB.pos = 27;
    }


  } else if (fsm.state == 6){ // averigua a cor

    if (MH.pos > 65){
      MH.pos --;
    }
    if (MR.pos < 100){
      MR.pos ++;
    } else if (MR.pos > 100){
      MR.pos --;
    }

    if (RED > GREEN && RED > BLUE && GREEN < 1.5*BLUE) {
        colors[0] = true;
        colors[1] = colors[2] = colors[3] = false;
        got_color = true;
      } else if (GREEN > RED && GREEN > BLUE) {
        colors[1] = true;
        colors[0] = colors[2] = colors[3] = false;
        got_color = true;
      } else if (BLUE > RED && BLUE > GREEN) {
        colors[2] = true;
        colors[0] = colors[1] = colors[3] = false;
        got_color = true;
      } else if (RED > GREEN && GREEN > BLUE) {
        colors[3] = true;
        colors[0] = colors[1] = colors[2] = false;
        got_color = true;
      }

  } else if (fsm.state == 7){ //largamos a peça no sítio certo dependendo da cor
    if (colors[0] == true){ // RED
      if (MR.pos < 130){
        MR.pos ++; 
      } else if (MR.pos == 130){
        MC.pos = 0;
        in_place = true;
      }

      if (MH.pos < 110){
        MH.pos ++;
      }

    } if(colors[1] == true){ // GREEN

      if (MR.pos < 150){
        MR.pos ++;
      } else if (MR.pos == 150){
        MC.pos = 0;
        in_place = true;
      }

      if (MH.pos < 110) MH.pos ++;
      if (MB.pos < 35) MB.pos ++;

    } if(colors[2] == true){ // BLUE

      if (MR.pos < 130){
        MR.pos ++;
      } else if (MR.pos == 130){
        MC.pos = 0;
        in_place = true;
      }

      if (MH.pos < 110) MH.pos ++;
      if (MB.pos > 10) MB.pos --;

    } if(colors[3] == true){ // YELLOW

      if (MR.pos < 130){
        MR.pos ++;
      } else if (MR.pos == 130){
        MC.pos = 0;
        in_place = true;
      }

      if (MH.pos < 110) MH.pos ++;
      if (MB.pos > 0) MB.pos --;

    }

    
  } else if (fsm.state == 8){ //voltar ao estado de procura de peça
    
    if (MH.pos < 120){
      MH.pos = MH.pos + 2;
    } else if (MH.pos >= 120){
      MH.pos = 120;
    }

    if (MR.pos > 60){
      MR.pos = MR.pos - 5;
    } else if (MR.pos <= 60){
      MR.pos = 60;
    }

    if (MB.pos < 50){
      MB.pos = MB.pos + 2;
    } else if (MB.pos >= 50){
      MB.pos = 50;
    }

  } else if (fsm.state == 9){
    M_base.detach(); 
    M_reach.detach (); 
    M_catch.detach();
    M_height.detach();
  }


}

void update_outputs(){
  M_base.write(int(MB.pos));
  M_reach.write(int(MR.pos));
  M_catch.write(int(MC.pos));
  M_height.write(int(MH.pos));
}



void setup() 
{ 
  M_base.attach(0, 544, 2344); // range 0(looking left) - 180(looking right)
  M_reach.attach(2, 544, 2344); // range 35(retracted) - 170(MH.dired)
  M_catch.attach(4, 544, 2344); // range 0(open) - 100(close)
  M_height.attach(6, 544, 2344); // range 40 (low) - 160 (high)

  // setting start state
  update_state(fsm, 0);

  // set activity to false (dont do things until told to)
  start_activity = false;

  interval = 40;
  loop_count = 0;

  delay(50);

  Serial.begin(115200);

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.begin();

  tof.setBus(&Wire);
  tof.setAddress(0x53);
  tof.setTimeout(500);
  
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }
  // Start new distance measure
  tof.startReadRangeMillimeters();

  while (!tcs.begin(41, &Wire1)) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  } 
}

void loop() 
{
  
  // use serial to make the robot start measuring

  uint8_t b;
  if (Serial.available()) { 
    b = Serial.read();       
    if (b == '+') start_activity = true;  
    if (b == '-') start_activity = false;  
  }  

  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    } 

    uint16_t r, g, b, c;
    getRawData_noDelay(&r, &g, &b, &c);

    RED = r;
    GREEN = g;
    BLUE = b;

    //Serial.print(" start activity: ");Serial.print(start_activity);Serial.print(" ");

    Serial.print(" Dist: ");Serial.print(distance, 3);Serial.print(" ");
    Serial.print(" Dist_OBJ: ");Serial.print(dist_to_reach, 3);Serial.print(" ");
    Serial.print(" Loop_Count: ");Serial.print(loop_count);Serial.print(" ");
    //Serial.print(" mean_dist: ");Serial.print(mean_dist, 3);Serial.print(" ");
    //Serial.print(" dist_to_reach: ");Serial.print(dist_to_reach, 3);Serial.print(" ");

    Serial.print(" State: ");Serial.print(fsm.state);Serial.print(" ");
    Serial.print(" Reach: ");Serial.print(MR.pos);Serial.print(" ");
    Serial.print(" Height: ");Serial.print(MH.pos);Serial.print(" ");
    Serial.print(" Catch: ");Serial.print(MC.pos);Serial.print(" ");
    Serial.print(" Base: ");Serial.print(MB.pos);Serial.print(" ");
    Serial.print(" RGB: ");Serial.print(RED);Serial.print(" ");Serial.print(GREEN);Serial.print(" ");Serial.print(BLUE);Serial.print(" ");

    if (colors[0] == true){
      Serial.print(" Color detected: Red " );
    } else if (colors[1] == true){
      Serial.print(" Color detected: Green ");
    } else if (colors[3]  == true){
      Serial.print(" Color detected: Yellow ");
    } else if (colors[2]  == true){
      Serial.print(" Color detected: Blue ");
    } else {
      Serial.print(" No color detected ");
    }
    
    Serial.print(" TIS: ");Serial.print(fsm.tis);

    Serial.println();
  
    unsigned long now = millis();  
    fsm.tis = now - fsm.tes;
    
    calc_next_state();

    update_state(fsm, fsm.new_state);

    calc_outputs();

    update_outputs();

    // Start new distance measure
    tof.startReadRangeMillimeters(); 
  }
}