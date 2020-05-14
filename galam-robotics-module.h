#include <stdint.h>
#include <stdio.h>
#define BUFFSIZE 32
#define NB_MAX_MSG 4
#define INIT 0
#define INIT_R 1
#define MSG_TO_SON 2
#define MSG_TO_SOURCE 3
#define TIME_OUT 500
#define UNKNOWN_ID 4
#define END_NODE 3
#define END_HEADER 3

int compareArrays(uint8_t *a, uint8_t *b, int size);
void incr_iterators (int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr);

struct Module
{
  uint8_t rx0[10][BUFFSIZE];
  int nb_rx0;
  uint8_t rx1[10][BUFFSIZE];
  int nb_rx1;
  uint8_t rx2[10][BUFFSIZE];
  int nb_rx2;

  uint8_t father_id;
  uint8_t son_ids[2];
  int son_nb;

  uint8_t msg_stored[3];
  uint8_t msg_to_store[3];
  uint8_t msgx0[NB_MAX_MSG][BUFFSIZE];
  uint8_t msgx1[NB_MAX_MSG][BUFFSIZE];
  uint8_t msgx2[NB_MAX_MSG][BUFFSIZE];

  void (*Store_Message)(struct Module *this, uint8_t *pData, uint8_t id);
  void (*emptyStorage)(struct Module *this, uint8_t id);
  void (*Handle_Message)(struct Module *this, uint8_t *pData, uint8_t id);
  void (*Handle_Message_init)(struct Module *this, uint8_t *pData, uint8_t id);
  void (*Handle_Message_init_r)(struct Module *this, uint8_t *pData, uint8_t id);
  void (*Send_init_r)(struct Module *this);
  void (*Handle_Message_to_son)(struct Module *this, uint8_t *pData, uint8_t id);
  void (*Send_Message_to_son)(struct Module *this);
  void (*Handle_Message_to_source)(struct Module *this, uint8_t *pData);

  struct Module* connections[3];
  uint8_t connections_ids[3];
  uint8_t (*Transmit)(struct Module *this, uint8_t id, uint8_t *pData);
  void (*Handle_All_Message)(struct Module *this);
};
extern const struct ModuleClass {
  struct Module (*new)();
} Module;
