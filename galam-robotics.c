#include "galam-robotics.h"

/*
Variables
Par convention, l'interface 0 est l'interface amont, et les interfaces avales sont les interfaces 1 et 2. 
Les messages en provenance de l'interface 0 seront donc recus dans le tableau rx0 et 
les messages à envoyer vers l'interface 0 seront stockés dans le tableau tx0. 
*/

uint8_t rx0[BUFFSIZE];
uint8_t rx1[BUFFSIZE];
uint8_t rx2[BUFFSIZE];

uint8_t father_id = UNKNOWN_ID;
uint8_t son_ids[] = {UNKNOWN_ID, UNKNOWN_ID};
int son_nb = 0;

uint8_t msg_stored[] = {0, 0, 0};
uint8_t msg_to_store[] = {0, 0, 0};
uint8_t msgx0[NB_MAX_MSG][BUFFSIZE];
uint8_t msgx1[NB_MAX_MSG][BUFFSIZE];
uint8_t msgx2[NB_MAX_MSG][BUFFSIZE];

/*
Fonction de stockage de messages
*/
void Store_Message(uint8_t *pData, uint8_t id)
{
    // on pointe vers le stockage correspondant a l'id
    uint8_t* msg_storage;
    if (id == 0)
    {
	msg_storage = &msgx0[msg_stored[id]][0];
    }
    else if (id == 1)
    {
	msg_storage = &msgx1[msg_stored[id]][0];
    }
    else if (id == 2)
    {
	msg_storage = &msgx2[msg_stored[id]][0];
    }

    // on copie les donnees
    for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
    {
	msg_storage[byte_i] = pData[byte_i];
    }

    // on actualise cette variable
    msg_stored[id]++;
}

void emptyStorage(uint8_t id)
{
    // on pointe vers le stockage correspondant a l'id
    uint8_t* msg_storage[NB_MAX_MSG];
    if (id == 0)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx0[msg_i][0];
	}
    }
    else if (id == 1)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx1[msg_i][0];
	}
    }
    else if (id == 2)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx2[msg_i][0];
	}
    }

    // on met des 0 partout
    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++) {
	for (int byte_i = 0; byte_i < BUFFSIZE - 1; byte_i++) {
	    msg_storage[byte_i] = 0;
	}
    }

    // on actualise cette variable
    msg_stored[id] = 0;
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
    uint8_t msg_type = ((pData[0])&0b11000000) >> 6;
    if (msg_type == INIT) // init
    {
	Handle_Message_init(pData, id);
    }
    else if (msg_type == INIT_R) // init_r
    {
	Handle_Message_init_r(pData, id);
    }
    else if (msg_type == MSG_TO_SON) // message to son
    {
	Handle_Message_to_son(pData, id);
    }
    else if (msg_type == MSG_TO_SOURCE) // message to source
    {
	Handle_Message_to_source(pData);
    }
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

    // on stocke le message
    Store_Message(pData, id);

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
    // tableau de stockage du message init_r a envoyer
    uint8_t msg_to_send[NB_MAX_MSG][BUFFSIZE];
    // iterateurs d'ecriture
    int write_msg_i = 0;
    int write_byte_i = 1;
    int write_offset = 6;

    // pour chaque fils
    for (int son_i = 0; son_i < son_nb; son_i++)
    {
	// on recupere son identifiant
	uint8_t id = son_ids[son_i];
	// on commence le message par son identifiant (voir algorithme)
	msg_to_send[write_msg_i][write_byte_i] += id << write_offset;
	// on met a jout les iterateurs d'ecriture
	incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);

	// iterateurs de lecture du message recu par ce fils
	int son_msg_i = 0;
	int son_byte_i = 1;
	int son_offset = 6;
	uint8_t and_op = 0b11000000;
	// tableau contenant le message recu par ce fils (on le fait pointer vers le bon
	// tableau de stockage)
	uint8_t* msg_storage[NB_MAX_MSG];
	if (id == 0)
	{
	    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	    {
		msg_storage[msg_i] = &msgx0[msg_i][0];
	    }
	}
	else if (id == 1)
	{
	    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	    {
		msg_storage[msg_i] = &msgx1[msg_i][0];
	    }
	}
	else if (id == 2)
	{
	    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	    {
		msg_storage[msg_i] = &msgx2[msg_i][0];
	    }
	}

	// on lit le message pour trouver sa fin grace a la profondeur dans l'arbre
	int depth = 1;
	while (depth > 0)
	{
	    uint8_t value = (msg_storage[son_msg_i][son_byte_i]&and_op) >> son_offset;
	    // pour chaque valeur qu'on lit, on l'ecrit dans le message a envoyer
	    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
	    incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
	    incr_iterators(&son_msg_i, &son_byte_i, &son_offset, &and_op, 2);

	    if (value == END_NODE) {depth--;} // on diminue la pronfondeur en fin de noeud
	    else {depth++;} // sinon on augmente
	}

	emptyStorage(id);
    }

    // derniere valeur pour le message a envoyer
    msg_to_send[write_msg_i][write_byte_i] += END_NODE << write_offset; 

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = INIT_R;
    uint8_t nb_msg = write_msg_i + 1;
    for (int i = 0; i <= write_msg_i; i++)
    {
	msg_to_send[i][0] = (msg_type << 6) + nb_msg;
	Transmit(father_id, msg_to_send[i], BUFFSIZE, TIME_OUT);
    }
}

/*
Fonction pour transmettre un message a un fils
*/
void Handle_Message_to_son(uint8_t *pData, uint8_t id)
{
    // on regarde combien de message on doit stocker
    if (msg_to_store[id] == 0)
    {
	msg_to_store[id] = (pData[0])&0b00111111;
    }

    // on stocke le message
    Store_Message(pData, id);

    if (msg_stored[id] == msg_to_store[id])
    {
	uint8_t next_id;
	if (father_id == 0)
	{
	    next_id = (msgx0[0][1]&0b11000000) >> 6;
	}
	else if (father_id == 1)
	{
	    next_id = (msgx1[0][1]&0b11000000) >> 6;
	}
	else if (father_id == 2)
	{
	    next_id = (msgx2[0][1]&0b11000000) >> 6;
	}

	if (next_id != END_HEADER)
	{
	    Send_Message_to_son();
	}
	else
	{
	    // TODO Read_Message
	}
    }
}

/*
Fonction d'envoi d'un message a un des fils
*/
void Send_Message_to_son()
{
    // tableau de stockage du message a envoyer
    uint8_t msg_to_send[NB_MAX_MSG][BUFFSIZE];
    // iterateurs d'ecriture
    int write_msg_i = 0;
    int write_byte_i = 1;
    int write_offset = 7;

    // iterateurs de lecture
    int read_msg_i = 0;
    int read_byte_i = 1;
    int read_offset = 6;
    uint8_t and_op = 0b10000000;
    // on recupere le message stocker en pointant vers le bon identifiant du pere
    uint8_t* msg_storage[NB_MAX_MSG];
    if (father_id == 0)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx0[msg_i][0];
	}
    }
    else if (father_id == 1)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx1[msg_i][0];
	}
    }
    else if (father_id == 2)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &msgx2[msg_i][0];
	}
    }

    // segment routing : il faut transmettre le message au prochain id
    uint8_t next_id = (msg_storage[read_msg_i][read_byte_i]&and_op) >> read_offset;
    incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

    while (read_msg_i < msg_stored[father_id]) // TODO: ameliorer ?
    {
	uint8_t value = (msg_storage[read_msg_i][read_byte_i]&and_op) >> read_offset;
	msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
	incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 1);
	incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1);
    }

    emptyStorage(father_id);

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = MSG_TO_SON;
    uint8_t nb_msg = write_msg_i + 1;
    for (int i = 0; i <= write_msg_i; i++)
    {
	msg_to_send[i][0] = (msg_type << 6) + nb_msg;
	Transmit(next_id, msg_to_send[i], BUFFSIZE, TIME_OUT);
    }
}

/*
Fonction pour transmettre un message a la source
*/
void Handle_Message_to_source(uint8_t *pData)
{
    Transmit(father_id, pData, BUFFSIZE, TIME_OUT);
    // TODO: fonction Send_Message_to_Source(unint8_t *pData, uint16_t length) qui decoupe et
    // envoie le message
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

/*
Fonction pour incrementer les iterateurs durant la lecture/ecriture de message
*/
void incr_iterators (int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr)
{
    if (*offset == 0) {
	*offset = 8 - incr;
	if (and_op != NULL)
	{
	    if (incr == 1)
	    {
		*and_op = 0b10000000;
	    }
	    else if (incr == 2)
	    {
		*and_op = 0b11000000;
	    }
	}

	if (*byte_i == BUFFSIZE - 1)
	{
	    *msg_i += 1;
	    *byte_i = 1;
	}
	else
	{
	    *byte_i += 1;
	}
    }
    else
    {
	*offset -= incr;
	if (and_op != NULL)
	{
	    *and_op = *and_op >> incr;
	}
    }
}
