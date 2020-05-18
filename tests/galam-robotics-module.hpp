#include <stdint.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#define NB_INTERFACE 3
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

class Module
{
  private:
    uint8_t received[NB_INTERFACE][NB_MAX_MSG][BUFFSIZE] = {0};
    int received_nb[3] = {0, 0, 0};

    uint8_t father_id = UNKNOWN_ID;
    uint8_t son_ids[2] = {UNKNOWN_ID, UNKNOWN_ID};
    int son_nb = 0;

    uint8_t msg_stored[3] = {0, 0, 0};
    uint8_t msg_to_store[3] = {0, 0, 0};
    uint8_t storage[NB_INTERFACE][NB_MAX_MSG][BUFFSIZE] = {0};
  
    Module* connections[3] = {NULL, NULL, NULL};
    uint8_t connections_other_side_id[3] = {UNKNOWN_ID, UNKNOWN_ID, UNKNOWN_ID};
    uint8_t son_ids_to_print[2] = {UNKNOWN_ID, UNKNOWN_ID};
    std::string state = "NI";
    std::string last_message = "no msg";
    
  public:
    int module_id = 0;
    Module ();

    // fonctions d'un module
    void Store_Message(uint8_t *pData, uint8_t id);
    void Empty_Storage(uint8_t id);
    void Handle_Message(uint8_t *pData, uint8_t id);
    void Handle_Message_init(uint8_t *pData, uint8_t id);
    void Handle_Message_init_r(uint8_t *pData, uint8_t id);
    void Send_init_r();
    void Handle_Message_to_Son(uint8_t *pData, uint8_t id);
    void Transfer_Message_to_Son();
    void Read_Message(uint8_t *pData);
    void Handle_Message_to_Source(uint8_t *pData);
    void Send_Message_to_Source(uint8_t *pData, uint16_t length);

    // fonctions ajoutees pour assurer les tests
    uint8_t get_random_id();
    void set_father(Module* father, uint8_t id_this_side, uint8_t id_father_side);
    void set_first_son(Module* son, uint8_t id_this_side, uint8_t id_son_side);
    void set_second_son(Module* son, uint8_t id_this_side, uint8_t id_son_side);
    void print(int depth = 0);
    int naming(int id = 0);
    uint8_t Transmit(uint8_t id, uint8_t *pData);
    void Handle_All_Message();
    uint8_t Send_init();
    void Send_Message(uint8_t entry_id, uint8_t *pData);
};
