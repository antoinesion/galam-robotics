#include "galam-robotics.h"

/*
Variables
Par convention, l'interface 0 est l'interface amont, et les interfaces avales sont les interfaces 1 et 2. 
Les messages en provenance de l'interface 0 seront donc recus dans le tableau rx0 et 
les messages à envoyer vers l'interface 0 seront stockés dans le tableau tx0. 
*/

uint8_t tx0[BUFFSIZE] ;
uint8_t rx0[BUFFSIZE] ;
uint8_t tx1[BUFFSIZE] ;
uint8_t rx1[BUFFSIZE] ;
uint8_t tx2[BUFFSIZE] ;
uint8_t rx2[BUFFSIZE] ;

uint8_t father_id = UNKNOWN_ID;
uint8_t son_ids[] = {UNKNOWN_ID, UNKNOWN_ID};
int son_nb = 0;

int init_r_depths[] = {1, 1, 1};

uint8_t msg_stored[] = {0, 0, 0};
uint8_t msg_to_store[] = {0, 0, 0};
uint8_t msgx0 [NB_MAX_MSG][BUFFSIZE - 1];
int msgx0_i = 0;
int msgx0_byte_i = 0;
int msgx0_bit_offset = 6;
uint8_t msgx1 [NB_MAX_MSG][BUFFSIZE - 1];
int msgx1_i = 0;
int msgx1_byte_i = 0;
int msgx1_bit_offset = 6;
uint8_t msgx2 [NB_MAX_MSG][BUFFSIZE - 1];
int msgx2_i = 0;
int msgx2_byte_i = 0;
int msgx2_bit_offset = 6;

/*
Fonction de stockage de messages, données dans pData jusqu'à byte_i / offset
*/
void Store_Message(uint8_t *pData, int id, int byte_i_end, int bit_offset_end)
    // TODO : stocker betement !
{
    uint8_t* msg_storage[NB_MAX_MSG];
    int* msg_i;
    int* msg_byte_i;
    int* msg_bit_offset;
    if (id == 0)
    {
	for (int i = 0; i < NB_MAX_MSG; i++)
	{
	    msg_storage[i] = msgx0[i];
	}
	msg_i = &msgx0_i;
	msg_byte_i = &msgx0_byte_i;
	msg_bit_offset = &msgx0_bit_offset;
    }
    else if (id == 1)
    {
	for (int i = 0; i < NB_MAX_MSG; i++)
	{
	    msg_storage[i] = msgx1[i];
	}
	msg_i = &msgx1_i;
	msg_byte_i = &msgx1_byte_i;
	msg_bit_offset = &msgx1_bit_offset;
    }
    else if (id == 2)
    {
	for (int i = 0; i < NB_MAX_MSG; i++)
	{
	    msg_storage[i] = msgx2[i];
	}
	msg_i = &msgx2_i;
	msg_byte_i = &msgx2_byte_i;
	msg_bit_offset = &msgx2_bit_offset;
    }

    int byte_i = 0;
    int bit_offset = 6;
    uint8_t and_op = 0b11000000;
    while (byte_i < byte_i_end || bit_offset < bit_offset_end) {
	msg_storage[*msg_i][*msg_byte_i] += (((pData[byte_i]&and_op) >> bit_offset) << *msg_bit_offset);

	
	if (bit_offset == 0) {
	    byte_i++;
	    and_op = 0b11000000;
	    bit_offset = 6;
	}
	else
	{
	    and_op = and_op >> 2;
	    bit_offset -= 2;
	}

	if (*msg_bit_offset == 0) {
	    (*msg_byte_i)++;
	    if (*msg_byte_i == BUFFSIZE)
	    {
		(*msg_i)++;
		*msg_byte_i = 0;
	    }
	    *msg_bit_offset = 6;
	}
	else
	{
	    *msg_bit_offset -= 2;
	}
    }

    msg_stored[id]++;
}

void emptyStorage(int id) {
    // TODO
}

/*
Fonction d'initialisation, est appelée une fois au démarrage du microcontroleur.
*/

void init()
{
    Receive_IT(0, rx0, BUFFSIZE) ;
    Receive_IT(1, rx1, BUFFSIZE) ;
    Receive_IT(2, rx2, BUFFSIZE) ;
}

/*
Fonction principale, la boucle infinie est le coeur du programme
*/

int main() {
    while(1 == 1)
    {

    }
}

uint8_t RxCallBack(uint8_t id)
{
    if(id == 0)
        {
            Handle_Message(rx0, id) ;
            Receive_IT(0, rx0, BUFFSIZE) ;
        }

    if(id == 1)
        {
            Handle_Message(rx1, id) ;
            Receive_IT(0, rx1, BUFFSIZE) ;
        }

    if(id == 2)
        {
            Handle_Message(rx2, id) ;
            Receive_IT(0, rx2, BUFFSIZE) ;
        }
    return 0;
}

void Handle_Message(uint8_t *pData, uint8_t id)
{
    uint8_t header = ((pData[0])&0b11000000) >> 6;
    if (header == 0) // init
    {
	Handle_Message_init(pData, id);
    }
    else if (header == 1) // init_r
    {
	Handle_Message_init_r(pData, id);
    }
    // ...
}

void Handle_Message_init(uint8_t *pData, uint8_t id)
{
    father_id = id;
    if (((pData[0])&0b00111111) == 1) // si ce n'est pas un message fantôme
    { 
	for (int trsmt_id = 0; trsmt_id < 3; trsmt_id++) // on transmet l'init aux fils
	{
	    if (trsmt_id != father_id) // pas au père
	    {
		uint8_t t = Transmit(trsmt_id, pData, BUFFSIZE, TIME_OUT);
		if (t == 1) // si la transmission a réussi, on ajoute le fils
		{
		    son_ids[son_nb] = trsmt_id;
		    msg_to_store[trsmt_id] = 1;
		    son_nb++;
		}

	    }
	}
    }

    if (son_nb == 0) { // si le noeud n'a pas de fils, on peut tout de suite envoyer l'init_r
	Send_init_r();
    }
}


void Handle_Message_init_r(uint8_t *pData, uint8_t id)
{
    // on regarde combien de message on doit stocker
    if (msg_to_store[id] == 1)
    {
	msg_to_store[id] = (pData[0])&0b00111111;
    }
    
    // TODO: stocker betement !
    // on lit pData pour retenir seulement ce qui est important (si c'est le dernier message,
    // sa longueur ne sera sans doute pas egale a BUFFSIZE)
    // ou retenir la profondeur du reseau a laquelle on s'est arreté au dernier message recu
    // par la meme interface
    uint8_t and_op = 0b11000000;
    int bit_offset = 6;
    int byte_i = 1;
    int depth = init_r_depths[id];
    while(depth > 0 && byte_i < BUFFSIZE)
    {
	if (((pData[byte_i])&and_op) >> bit_offset == END_NODE)
	{
	    depth--;
	}
	else
	{
	    depth++;
	}

	if (bit_offset == 0) {
	    byte_i++;
	    and_op = 0b11000000;
	    bit_offset = 6;
	}
	else
	{
	    and_op = and_op >> 2;
	    bit_offset -= 2;
	}
    }


    // on stocke le message
    Store_Message(pData, id, byte_i, bit_offset);

    if (compareArrays(msg_stored, msg_to_store, 3)) // si on a recu tous les messages de init_r
    {
	Send_init_r();
    }

}


/*
Fonction d'envoi d'init_r
*/
void Send_init_r()
{
    if (son_nb == 0)
    {

    }
}

/*
Fonction pour transmettre un message a un fils
*/
void Handle_Message_to_son(uint8_t *pData, uint8_t id)
{
    // TODO : stocker les messages et quand on a tout recu appeler une fonction Send_Message_to_son()
}

/*
Fonction pour transmettre un message au pere
*/
void Handle_Message_to_father(uint8_t *pData)
{
    // TODO : directment transmettre au pere, meme pas besoin de stocker ?
}

/*
Fonction de comparaison de tableaux
*/
int compareArrays(uint8_t *a, uint8_t *b, int size)
{
    for (int i = 0; i < size; i++)
    {
	if (a[i] != b[i])
	{
	    return 0;
	}
    }
    return 1;
}
