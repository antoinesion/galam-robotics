#include "galam-robotics-module.hpp"
#include <bitset>

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

Module::Module() {}

void Module::Store_Message(uint8_t *pData, uint8_t itf)
{
    // on copie les donnees sur le bon tableau de stockage
    for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
    {
	storage[itf][msg_stored[itf]][byte_i] = pData[byte_i];

	/* std::bitset<8> byte (pData[byte_i]);
	 * std::cout << byte << " ";
	 * if (byte_i % 4 == 3)
	 * {
	 *   std::cout << std::endl;
         * }  */
    }

    // on actualise cette variable
    msg_stored[itf]++;
}

void Module::Empty_Storage(uint8_t itf)
{
    // on met des 0 partout
    for (int msg_i = 0; msg_i < NB_MAX_SBMSG; msg_i++) {
	for (int byte_i = 0; byte_i < BUFFSIZE - 1; byte_i++) {
	    storage[itf][msg_i][byte_i] = 0;
	}
    }

    // on actualise cette variable
    msg_stored[itf] = 0;
    msg_to_store[itf] = 0;
}

void Module::Handle_Message(uint8_t *pData, uint8_t itf)
{
    uint8_t msg_type = ((pData[0])&0b11000000) >> 6;
    if (msg_type == INIT) // init
    {
	Handle_Message_init(pData, itf);
    }
    else if (msg_type == INIT_R) // init_r
    {
	Handle_Message_init_r(pData, itf);
    }
    else if (msg_type == MSG_TO_SON) // message to son
    {
	Handle_Message_to_Son(pData, itf);
    }
    else if (msg_type == MSG_TO_SOURCE) // message to source
    {
	Handle_Message_to_Source(pData);
    }
}

void Module::Handle_Message_init(uint8_t *pData, uint8_t itf)
{
    father_itf = itf;
    if (((pData[0])&0b00111111) == 1) // si ce n'est pas un message fantôme
    { 
	for (int trsmt_itf = 0; trsmt_itf < 3; trsmt_itf++) // on transmet l'init aux fils
	{
	    if (trsmt_itf != father_itf) // pas au père
	    {
		uint8_t t = Transmit(trsmt_itf, pData);
		if (t == 1) // si la transmission a réussi, on ajoute le fils
		{
		    son_itfs[son_nb] = trsmt_itf;
		    msg_to_store[trsmt_itf] = 1;
		    son_nb++;
		}

	    }
	}
    }

    state = "I";

    if (son_nb == 0) { // si le noeud n'a pas de fils, on peut tout de suite envoyer l'init_r
	Send_init_r();
    }
}

void Module::Handle_Message_init_r(uint8_t *pData, uint8_t itf)
{
    // on regarde combien de message on doit stocker
    if (msg_to_store[itf] == 1)
    {
	msg_to_store[itf] = (pData[0])&0b00111111;
    }

    // on stocke le message
    Store_Message(pData, itf);

    if (compareArrays(msg_stored, msg_to_store, 3)) // si on a recu tous les messages de init_r
    {
	Send_init_r();
    }
}

void Module::Send_init_r()
{
    // tableau de stockage du message init_r a envoyer
    uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE];
    for (int msg_i = 0; msg_i < NB_MAX_SBMSG; msg_i++)
    {
      for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
      {
	msg_to_send[msg_i][byte_i] = 0;
      }
    }

    // iterateurs d'ecriture
    int write_msg_i = 0;
    int write_byte_i = 1;
    int write_offset = 6;

    // pour chaque fils
    for (int son_i = 0; son_i < son_nb; son_i++)
    {
	// on recupere son identifiant
	uint8_t itf = son_itfs[son_i];
	// on commence le message par son itfentifiant (voir algorithme)
	msg_to_send[write_msg_i][write_byte_i] += itf << write_offset;
	// on met a jout les iterateurs d'ecriture
	incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);

	// iterateurs de lecture du message recu par ce fils
	int son_msg_i = 0;
	int son_byte_i = 1;
	int son_offset = 6;
	uint8_t and_op = 0b11000000;
	
	// on lit le message pour trouver sa fin grace a la profondeur dans l'arbre
	int depth = 1;
	while (depth > 0)
	{ 
	    uint8_t value = (storage[itf][son_msg_i][son_byte_i]&and_op) >> son_offset;
	    // pour chaque valeur qu'on lit, on l'ecrit dans le message a envoyer
	    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
	    incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
	    incr_iterators(&son_msg_i, &son_byte_i, &son_offset, &and_op, 2);

	    if (value == END_NODE) {depth--;} // on diminue la pronfondeur en fin de noeud
	    else {depth++;} // sinon on augmente
	}

	Empty_Storage(itf);
    }

    // derniere valeur pour le message a envoyer
    msg_to_send[write_msg_i][write_byte_i] += END_NODE << write_offset; 

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = INIT_R;
    uint8_t nb_msg = write_msg_i + 1;
    for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    {
	msg_to_send[msg_i][0] = (msg_type << 6) + nb_msg;
	Transmit(father_itf, msg_to_send[msg_i]);
    }

    state = "IR";
}

void Module::Handle_Message_to_Son(uint8_t *pData, uint8_t itf)
{
    // on regarde combien de message on doit stocker
    if (msg_to_store[itf] == 0)
    {
	msg_to_store[itf] = (pData[0])&0b00111111;
    }

    // on stocke le message
    Store_Message(pData, itf);

    if (msg_stored[itf] == msg_to_store[itf])
    {
	uint8_t next_itf = (storage[father_itf][0][1]&0b11000000) >> 6;
	if (next_itf != END_HEADER)
	{
	    Transfer_Message_to_Son();
	}
	else
	{
	    uint8_t message[(BUFFSIZE - 1) * NB_MAX_SBMSG] = {0};
	    int read_msg_i = 0;
	    int read_byte_i = 1, write_byte_i = 0;
	    int read_offset = 4, write_offset = 6;
	    uint8_t and_op = 0b00110000;

	    while (read_msg_i < msg_stored[itf])
	    {
	      uint8_t value = (storage[itf][read_msg_i][read_byte_i]&and_op) >> read_offset;
	      message[write_byte_i] += value << write_offset;
	      incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);
	      if (write_offset == 0)
	      {
		write_offset = 6;
		write_byte_i++;
	      }
	      else
	      {
		write_offset -= 2;
	      }
	    }

	    Read_Message(message);
	    Empty_Storage(father_itf);
	}
    }
}

void Module::Transfer_Message_to_Son()
{
    // tableau de stockage du message a envoyer
    uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};
    // iterateurs d'ecriture
    int write_msg_i = 0;
    int write_byte_i = 1;
    int write_offset = 6;

    // iterateurs de lecture
    int read_msg_i = 0;
    int read_byte_i = 1;
    int read_offset = 6;
    uint8_t and_op = 0b11000000;
    
    // segment routing : il faut transmettre le message au prochain itf
    uint8_t next_itf = (storage[father_itf][read_msg_i][read_byte_i]&and_op) >> read_offset;
    incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

    while (read_msg_i < msg_stored[father_itf]) // TODO: ameliorer avec un header applicatif qui donne la longueur du message ?
    {
	uint8_t value = (storage[father_itf][read_msg_i][read_byte_i]&and_op) >> read_offset;
	msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
	incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
	incr_iterators(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);
    }

    Empty_Storage(father_itf);

    // on transmet les messages avec le bon header a chaque fois
    uint8_t msg_type = MSG_TO_SON;
    uint8_t nb_msg = write_msg_i + 1;
    for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    {
	msg_to_send[msg_i][0] = (msg_type << 6) + nb_msg;
	Transmit(next_itf, msg_to_send[msg_i]);
    }
}

void Module::Read_Message(uint8_t *pData)
{
  last_message = "";
  uint8_t length = pData[0];
  for (int byte_i = 1; byte_i < length + 1 ; byte_i++)
  {
    last_message.push_back((char) pData[byte_i]);

    /* std::bitset<8> byte (pData[byte_i]);
     * std::cout << byte << " ";
     * if (byte_i % 4 == 3)
     * {
     *   std::cout << std::endl;
     * } */
  }
  if (last_message.find("id=") == 0)
  {
    id = std::stoi(last_message.substr(3));
  }

  std::string text = std::to_string(id) + ": bien recu";
  const uint16_t text_len = text.size();
  uint8_t message[text_len + 1];
  message[0] = text_len;
  for (int i = 1; i < text_len + 1; i++)
  {
    message[i] = (uint8_t) text[i - 1];
  }
  Send_Message_to_Source(message, text_len+1);
}

void Module::Handle_Message_to_Source(uint8_t *pData)
{
    Transmit(father_itf, pData);
}

void Module::Send_Message_to_Source(uint8_t *pData, uint16_t length)
{
  uint8_t message[NB_MAX_SBMSG][BUFFSIZE] = {0};
  int write_msg_i;
  int write_byte_i;

  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
  {
    write_msg_i = read_byte_i / (BUFFSIZE - 1);
    write_byte_i = read_byte_i % (BUFFSIZE - 1) + write_msg_i + 1;
    message[write_msg_i][write_byte_i] = pData[read_byte_i];
  }

  int nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
  {
    message[msg_i][0] = (MSG_TO_SOURCE << 6) + nb_msg;
    Transmit(father_itf, message[msg_i]);
  }
}


uint8_t Module::Transmit(uint8_t itf, uint8_t *pData)
{
  if (connections[itf] == NULL)
  {
    if (itf == father_itf)
    {
      if ((pData[0]&0b11000000) >> 6 == INIT_R)
      {
	
	std::cout << "init_r message received (written in init_r.txt)" << std::endl;
	std::ofstream file;
	file.open("init_r.txt", std::ios::app);
	int msg_i = 0;
	int byte_i = 1;
	int offset = 6;
	uint8_t and_op = 0b11000000;
	while (msg_i == 0)
	{
	  int value = ((pData[byte_i]&and_op) >> offset);
	  std::cout << value;
	  file << value;
	  incr_iterators(&msg_i, &byte_i, &offset, &and_op, 2);
	}
	std::cout << std::endl;
	file.close();
      }
      if ((pData[0]&0b11000000) >> 6 == MSG_TO_SOURCE)
      {
	uint8_t length = pData[1];
	for (int byte_i = 2; byte_i < length + 2 ; byte_i++)
	{
	  std::cout << (char) pData[byte_i];
	}
	std::cout << std::endl;
      }
    }
    return 0;
  }
  else
  {
    Module* module_to_trsmt = connections[itf];
    // std::cout << "transmit (" << unsigned(id) << "): " << unsigned(itf) << std::endl;
    for (int byte_i = 0 ; byte_i < BUFFSIZE ; byte_i++)
    {
      module_to_trsmt->received[connections_other_side_itf[itf]][
	module_to_trsmt->received_nb[connections_other_side_itf[itf]]][byte_i] = pData[byte_i];

      /* std::bitset<8> byte (pData[byte_i]);
       * std::cout << byte << " ";
       * if (byte_i % 4 == 3)
       * {
       *   std::cout << std::endl;
       * }   */

    }
    
    module_to_trsmt->received_nb[connections_other_side_itf[itf]]++;
    return 1;
  }
}

void Module::Handle_All_Message()
{
  last_message = "no msg";
  for (int itf = 0 ; itf < NB_ITF ; itf++)
  {
    for (int msg_i = 0 ; msg_i < received_nb[itf] ; msg_i++)
    {
      Handle_Message(received[itf][msg_i], itf);
    }
    received_nb[itf] = 0;
  }
}

uint8_t Module::get_random_itf()
{
  uint8_t random_itf = rand() % 3;
  while (connections_other_side_itf[random_itf] != UNKNOWN_ITF)
  {
    random_itf = rand() % 3;
  }
  return random_itf;
}

void Module::set_father(Module* father, uint8_t itf_this_side, uint8_t itf_father_side)
{
  connections[itf_this_side] = father;
  connections_other_side_itf[itf_this_side] = itf_father_side;
}

void Module::set_first_son(Module* son, uint8_t itf_this_side, uint8_t itf_son_side)
{
  connections[itf_this_side] = son;
  connections_other_side_itf[itf_this_side] = itf_son_side;
  son_itfs_to_print[0] = itf_this_side;
}

void Module::set_second_son(Module* son, uint8_t itf_this_side, uint8_t itf_son_side)
{
  connections[itf_this_side] = son;
  connections_other_side_itf[itf_this_side] = itf_son_side;
  son_itfs_to_print[1] = itf_this_side;
}

void Module::print(int depth)
{
  std::string space (9*depth, ' ');
  std::cout << space;
  std::cout << unsigned(connections_other_side_itf[father_itf]) << "->" << unsigned(father_itf) << " ";
  std::cout << "(" << unsigned(id) << ") ";
  std::cout << state << " ";
  std::cout << received_nb[0] + received_nb[1] + received_nb[2] << "M ";
  std::cout << ": " << last_message;
  std::cout << std::endl;
  if (son_itfs_to_print[0] != UNKNOWN_ITF)
  {
    connections[son_itfs_to_print[0]]->print(depth + 1);
  }
    if (son_itfs_to_print[1] != UNKNOWN_ITF)
  {
    connections[son_itfs_to_print[1]]->print(depth + 1);
  } 
}

uint8_t Module::Send_init()
{
  uint8_t open_itf = this->get_random_itf();
  uint8_t init_msg[BUFFSIZE] = {0};
  init_msg[0] = 0b00000001;
  
  for (int byte_i = 0 ; byte_i < BUFFSIZE ; byte_i++)
  {
    received[open_itf][received_nb[open_itf]][byte_i] = init_msg[byte_i];
  }
  received_nb[open_itf]++;
  
  return open_itf;
}

void Module::Send_Message(uint8_t entry_itf, uint8_t *pData)
{
  for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
  {
    received[entry_itf][received_nb[entry_itf]][byte_i] = pData[byte_i];
  }
  received_nb[entry_itf]++;
}
