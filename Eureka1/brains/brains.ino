
#include <SoftwareSerial.h>

#define serialRX 1
#define serialTX 2
#define serialControl 3
#define serialTransmit HIGH
#define serialReceive LOW

// ~~~~~~~~~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~~~~~~~~
#define PRESSURANT 1 // for pressurant motor controller 
#define TANKS 2 // for tanks motor controller
#define MOTORA 3
#define MOTORB 4

#define N_LOX_TRANSDUCER 11 //high
#define N_PROP_TRANSDUCER 12 //high
#define LOX_TRANSDUCER 13 //low
#define PROP_TRANSDUCER 14 //low



int n_lox_pressure = 0;
int n_prop_pressure = 0;

int lox_pressure = 0;
int prop_pressure = 0;

SoftwareSerial serial(serialRX, serialTX);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // read pressure data
  read_all_pressure_values();
  
  // calculate desired flow rate
  
  // calculate appropriate control for desired flow rate

  // apply control

  // transmit and store state
  // @SQ networking stuff
  // special board for sd storage

}

int desired_choke_area_given_pressure_differential(int curr_p_tank, int curr_p_n2){
  
}

bool adjust_motor(int board_id, int motor_id, int amount){
  send_message([board_id, amount, motor_id]);
}



void read_all_pressure_values(){
  n_lox_pressure = read_pressure(N_LOX_TRANSDUCER);
  n_lox_pressure = high_pressure_sensor_conversion(n_lox_pressure);
  
  n_prop_pressure = read_pressure(N_PROP_TRANSDUCER);
  n_prop_pressure = high_pressure_sensor_conversion(n_prop_pressure);
  
  lox_pressure = read_pressure(LOX_TRANSDUCER);
  lox_pressure = low_pressure_sensor_conversion(lox_pressure);
  
  prop_pressure = read_pressure(PROP_TRANSDUCER);
  prop_pressure = low_pressure_sensor_conversion(prop_pressure);
}

int _read_pressure(int sensor_num){ // modify this using general communications method.
  digitalWrite(serialControl, serialTransmit);
  serial.write(sensor_num);
  digitalWrite(serialControl, serialReceive);
  int pressure_val = serial.read();
  return pressure_val;
}

float low_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = ;
  return pressure;
}

float high_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = ;
  return pressure;
}

#define desired_exit_mass_flow_rate_pressurant = 0.02938; //kg / s
#define BETA = 5.78396 * pow(10, -6);

float curr_mass_flow_rate(float curr_pressure, int tank_id){
  if (tank_id == LOX){
    int motor_position = lox_pressurant_motor_position;
  } else if (tank_id == PROPELLANT){
    int motor_position = prop_pressurant_motor_position;
  }
  float CdA = cda(curr_pressure, motor_position);
  return CdA * sqrt(BETA * pow(curr_pressure, 2));
}

float cda(float pressure, float motor_position){
  return 0;
}

table CdA_pressure = createTable(200);



// ~~~~~~~~~~~~~~~~~~~HASH MAP SOURCE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

struct node{
    int key;
    int val;
    struct node *next;
};
struct table{
    int size;
    struct node **list;
};
struct table *createTable(int size){
    struct table *t = (struct table*)malloc(sizeof(struct table));
    t->size = size;
    t->list = (struct node**)malloc(sizeof(struct node*)*size);
    int i;
    for(i=0;i<size;i++)
        t->list[i] = NULL;
    return t;
}
int hashCode(struct table *t,int key){
    if(key<0)
        return -(key%t->size);
    return key%t->size;
}
void insert(struct table *t,int key,int val){
    int pos = hashCode(t,key);
    struct node *list = t->list[pos];
    struct node *newNode = (struct node*)malloc(sizeof(struct node));
    struct node *temp = list;
    while(temp){
        if(temp->key==key){
            temp->val = val;
            return;
        }
        temp = temp->next;
    }
    newNode->key = key;
    newNode->val = val;
    newNode->next = list;
    t->list[pos] = newNode;
}
int lookup(struct table *t,int key){
    int pos = hashCode(t,key);
    struct node *list = t->list[pos];
    struct node *temp = list;
    while(temp){
        if(temp->key==key){
            return temp->val;
        }
        temp = temp->next;
    }
    return -1;
}
