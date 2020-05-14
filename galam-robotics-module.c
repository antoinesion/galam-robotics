#include "galam-robotics-module.h"

static void Store_Message(struct Module *this, uint8_t *pData, uint8_t id) {
  // on pointe vers le stockage correspondant a l'id
  uint8_t* msg_storage;
  if (id == 0)
  {
      msg_storage = &(this->msgx0[this->msg_stored[id]][0]);
  }
  else if (id == 1)
  {
      msg_storage = &(this->msgx1[this->msg_stored[id]][0]);
  }
  else if (id == 2)
  {
      msg_storage = &(this->msgx2[this->msg_stored[id]][0]);
  }

  // on copie les donnees
  for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
  {
      msg_storage[byte_i] = pData[byte_i];
  }

  // on actualise cette variable
  this->msg_stored[id]++;
}

static void emptyStorage(struct Module *this, uint8_t id)
{
    // on pointe vers le stockage correspondant a l'id
    uint8_t* msg_storage[NB_MAX_MSG];
    if (id == 0)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx0[msg_i][0]);
	}
    }
    else if (id == 1)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx1[msg_i][0]);
	}
    }
    else if (id == 2)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx2[msg_i][0]);
	}
    }

    // on met des 0 partout
    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++) {
	for (int byte_i = 0; byte_i < BUFFSIZE - 1; byte_i++) {
	    msg_storage[byte_i] = 0;
	}
    }

    // on actualise cette variable
    this->msg_stored[id] = 0;
}

static void Handle_Message(struct Module *this, uint8_t *pData, uint8_t id)
{
    uint8_t msg_type = ((pData[0])&0b11000000) >> 6;
    if (msg_type == INIT) // init
    {
	this->Handle_Message_init(this, pData, id);
    }
    else if (msg_type == INIT_R) // init_r
    {
	this->Handle_Message_init_r(this, pData, id);
    }
    else if (msg_type == MSG_TO_SON) // message to son
    {
	this->Handle_Message_to_son(this, pData, id);
    }
    else if (msg_type == MSG_TO_SOURCE) // message to source
    {
	this->Handle_Message_to_source(this, pData);
    }
}

static void Handle_Message_init(struct Module *this, uint8_t *pData, uint8_t id)
{
    this->father_id = id;
    if (((pData[0])&0b00111111) == 1) // si ce n'est pas un message fantôme
    { 
	for (int trsmt_id = 0; trsmt_id < 3; trsmt_id++) // on transmet l'init aux fils
	{
	    if (trsmt_id != this->father_id) // pas au père
	    {
		uint8_t t = this->Transmit(this, trsmt_id, pData);
		if (t == 1) // si la transmission a réussi, on ajoute le fils
		{
		    this->son_ids[this->son_nb] = trsmt_id;
		    this->msg_to_store[trsmt_id] = 1;
		    this->son_nb++;
		}

	    }
	}
    }

    if (this->son_nb == 0) { // si le noeud n'a pas de fils, on peut tout de suite envoyer l'init_r
	this->Send_init_r(this);
    }
}

static void Handle_Message_init_r(struct Module *this, uint8_t *pData, uint8_t id)
{
    // on regarde combien de message on doit stocker
    if (this->msg_to_store[id] == 1)
    {
	this->msg_to_store[id] = (pData[0])&0b00111111;
    }

    // on stocke le message
    this->Store_Message(this, pData, id);

    if (compareArrays(this->msg_stored, this->msg_to_store, 3)) // si on a recu tous les messages de init_r
    {
	this->Send_init_r(this);
    }
}

static void Send_init_r(struct Module *this)
{
    // tableau de stockage du message init_r a envoyer
    uint8_t msg_to_send[NB_MAX_MSG][BUFFSIZE];
    // iterateurs d'ecriture
    int write_msg_i = 0;
    int write_byte_i = 1;
    int write_offset = 6;

    // pour chaque fils
    for (int son_i = 0; son_i < this->son_nb; son_i++)
    {
	// on recupere son identifiant
	uint8_t id = this->son_ids[son_i];
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
		msg_storage[msg_i] = &(this->msgx0[msg_i][0]);
	    }
	}
	else if (id == 1)
	{
	    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	    {
		msg_storage[msg_i] = &(this->msgx1[msg_i][0]);
	    }
	}
	else if (id == 2)
	{
	    for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	    {
		msg_storage[msg_i] = &(this->msgx2[msg_i][0]);
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

	this->emptyStorage(this, id);
    }

    // derniere valeur pour le message a envoyer
    msg_to_send[write_msg_i][write_byte_i] += END_NODE << write_offset; 

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = INIT_R;
    uint8_t nb_msg = write_msg_i + 1;
    for (int i = 0; i <= write_msg_i; i++)
    {
	msg_to_send[i][0] = (msg_type << 6) + nb_msg;
	this->Transmit(this, this->father_id, msg_to_send[i]);
    }
}

static void Handle_Message_to_son(struct Module *this, uint8_t *pData, uint8_t id)
{
    // on regarde combien de message on doit stocker
    if (this->msg_to_store[id] == 0)
    {
	this->msg_to_store[id] = (pData[0])&0b00111111;
    }

    // on stocke le message
    this->Store_Message(this, pData, id);

    if (this->msg_stored[id] == this->msg_to_store[id])
    {
	uint8_t next_id;
	if (this->father_id == 0)
	{
	    next_id = (this->msgx0[0][1]&0b11000000) >> 6;
	}
	else if (this->father_id == 1)
	{
	    next_id = (this->msgx1[0][1]&0b11000000) >> 6;
	}
	else if (this->father_id == 2)
	{
	    next_id = (this->msgx2[0][1]&0b11000000) >> 6;
	}

	if (next_id != END_HEADER)
	{
	    this->Send_Message_to_son(this);
	}
	else
	{
	    // TODO Read_Message
	}
    }
}

static void Send_Message_to_son(struct Module *this)
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
    if (this->father_id == 0)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx0[msg_i][0]);
	}
    }
    else if (this->father_id == 1)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx1[msg_i][0]);
	}
    }
    else if (this->father_id == 2)
    {
	for (int msg_i = 0; msg_i < NB_MAX_MSG; msg_i++)
	{
	    msg_storage[msg_i] = &(this->msgx2[msg_i][0]);
	}
    }

    // segment routing : il faut transmettre le message au prochain id
    uint8_t next_id = (msg_storage[read_msg_i][read_byte_i]&and_op) >> read_offset;
    incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

    while (read_msg_i < this->msg_stored[this->father_id]) // TODO: ameliorer ?
    {
	uint8_t value = (msg_storage[read_msg_i][read_byte_i]&and_op) >> read_offset;
	msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
	incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 1);
	incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1);
    }

    this->emptyStorage(this, this->father_id);

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = MSG_TO_SON;
    uint8_t nb_msg = write_msg_i + 1;
    for (int i = 0; i <= write_msg_i; i++)
    {
	msg_to_send[i][0] = (msg_type << 6) + nb_msg;
	this->Transmit(this, next_id, msg_to_send[i]);
    }
}

static uint8_t Transmit(struct Module *this, uint8_t id, uint8_t *pData)
{
  if (this->connections_ids[id] == UNKNOWN_ID)
  {
    return 0;
  }
  else
  {
    struct Module* module = this->connections[id];
    for (int byte_i = 0 ; byte_i < BUFFSIZE ; byte_i++)
    {
      if (this->connections_ids[id] == 0)
      {
	module->rx0[module->nb_rx0][byte_i] = pData[byte_i];
      }
      else if (this->connections_ids[id] == 1)
      {
	module->rx1[module->nb_rx1][byte_i] = pData[byte_i];
      }
      else if (this->connections_ids[id] == 2)
      {
	module->rx2[module->nb_rx2][byte_i] = pData[byte_i];
      }
    }
    
    if (this->connections_ids[id] == 0)
    {
      module->nb_rx0++;
    }
    else if (this->connections_ids[id] == 1)
    {
      module->nb_rx1++;
    }
    else if (this->connections_ids[id] == 2)
    {
      module->nb_rx2++;
    }

    return 1;
  }
}

static void Handle_All_Message(struct Module *this)
{
  for (int msg_i = 0 ; msg_i < this->nb_rx0 ; msg_i++)
  {
    this->Handle_Message(this, &(this->rx0[msg_i][0]), 0);
  }
  this->nb_rx0 = 0;
  
  for (int msg_i = 0 ; msg_i < this->nb_rx1 ; msg_i++)
  {
    this->Handle_Message(this, &(this->rx1[msg_i][0]), 1);
  }
  this->nb_rx1 = 0;
  
  for (int msg_i = 0 ; msg_i < this->nb_rx2 ; msg_i++)
  {
    this->Handle_Message(this, &(this->rx2[msg_i][0]), 2);
  }
  this->nb_rx2 = 0;
}

static struct Module new() {
  return (struct Module) {
    .father_id = UNKNOWN_ID,
    .son_ids = {UNKNOWN_ID, UNKNOWN_ID},
    .connections_ids = {UNKNOWN_ID, UNKNOWN_ID, UNKNOWN_ID},
    .Store_Message = &Store_Message,
    .emptyStorage = &emptyStorage,
    .Handle_Message = &Handle_Message,
    .Handle_Message_init = &Handle_Message_init,
    .Handle_Message_init_r = &Handle_Message_init_r,
    .Send_init_r = &Send_init_r,
    .Handle_Message_to_son = &Handle_Message_to_son,
    .Send_Message_to_son = &Send_Message_to_son
  };
}
const struct ModuleClass Module={.new=&new};

