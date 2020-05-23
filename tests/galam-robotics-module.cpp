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

void incr_indexes(int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr)
{
  if (*offset == 0)
  {
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

void Module::Store_Message(uint8_t itf, uint8_t *pData)
{
  // on copie les données sur le tableau de stockage correspondant à l'interface itf
  for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
  {
    storage[itf][msg_stored[itf]][byte_i] = pData[byte_i];
  }

  // on actualise cette variable
  msg_stored[itf]++;
}

void Module::Empty_Storage(uint8_t itf)
{
  // on supprime toutes les données stockées dans le tableau de stockage correspondant
  // à l'interface itf
  for (int msg_i = 0; msg_i < NB_MAX_SBMSG; msg_i++)
  {
    for (int byte_i = 0; byte_i < BUFFSIZE - 1; byte_i++)
    {
      storage[itf][msg_i][byte_i] = 0;
    }
  }

  // on actualise ces variables
  msg_stored[itf] = 0;
  msg_to_store[itf] = 0;
}

void Module::Handle_Message(uint8_t itf, uint8_t *pData)
{
  // le type du message est écrit sur les deux premiers bits du header
  // on recupère donc ce numéro codant ce type
  uint8_t msg_type = ((pData[0]) & 0b11000000) >> 6;

  // on redirige vers la bonne fonction
  if (msg_type == INIT) // init
  {
    Handle_Message_init(itf, pData);
  }
  else if (msg_type == INIT_R) // init_r
  {
    Handle_Message_init_r(itf, pData);
  }
  else if (msg_type == MSG_TO_MODULE) // message to module
  {
    Handle_Message_to_Module(pData);
  }
  else if (msg_type == MSG_TO_SOURCE) // message to source
  {
    Handle_Message_to_Source(pData);
  }
}

void Module::Handle_Message_init(uint8_t itf, uint8_t *pData)
{
  // comme le message init provient toujours de la source, l'interface depuis laquelle on recoit
  // l'init est donc l'interface du père du module (dans l'arbre)
  father_itf = itf;

  if (((pData[0]) & 0b00111111) == 1)
    // les 6 derniers bits de l'header code le nombre de sous-messages
    // naturellement, l'init (qui ne transmet pas de données particulières)
    // est codé sur un seul sous-message. Ainsi vérifier cela permet de s'assurer
    // que le message n'est pas un message fantôme
  {
    for (int itf = 0; itf < NB_ITF; itf++)
      // on parcourt les interfaces pour transmettre l'init
    {
      if (itf != father_itf) // on n'envoie pas l'init au père
      {
	// on transmet l'init
	uint8_t t = Transmit(itf, pData);

	if (t == 1) // si la transmission a reussi
	{
	  // on ajoute cette interface au tableau des interfaces fils
	  son_itfs[son_nb] = itf;
	  // on actualise cette variable
	  son_nb++;

	  // on doit donc attendre au moins un message d'init_r de la part de cette interface
	  msg_to_store[itf] = 1;
	}
      }
    }
  }

  state = "I"; // TODO: supprimer

  if (son_nb == 0)
    // si le noeud n'a pas de fils, on envoie tout de suite l'init_r
  {
    Send_init_r();
  }
}

void Module::Handle_Message_init_r(uint8_t itf, uint8_t *pData)
{
  // les 6 derniers bits du header code le nombre de sous-messages
  // ainsi, on regarde combien de sous-messages on doit stocker
  if (msg_to_store[itf] == 1)
  {
    msg_to_store[itf] = (pData[0]) & 0b00111111;
  }

  // on stocke le sous-message
  Store_Message(itf, pData);

  if (compareArrays(msg_stored, msg_to_store, 3))
    // on compare les tableaux du nombre de sous-messages stockés par interface
    // avec celui du nombre de sous-message que l'on doit stocker par interface
  {
    // si les deux sont égaux, on envoie l'init_r
    Send_init_r();
  }
}

void Module::Send_init_r()
{
  // tableau de stockage du message init_r à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 1;
  int write_offset = 6;

  // pour chaque fils
  for (int son_i = 0; son_i < son_nb; son_i++)
  {
    // on recupère son interface
    uint8_t itf = son_itfs[son_i];
    // on commence le message par son interface (algorithme d'init_r)
    msg_to_send[write_msg_i][write_byte_i] += itf << write_offset;
    // on met à jour les indices d'écriture
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);

    // indices de lecture du message recu par ce fils
    int read_msg_i = 0;
    int read_byte_i = 1;
    int read_offset = 6;
    uint8_t and_op = 0b11000000;

    // on lit le message pour trouver sa fin grâce à la profondeur dans l'arbre
    int depth = 1;
    // on s'arrête lorsque que la profondeur est nulle
    // i.e. on a parcouru tout le sous-arbre
    while (depth > 0)
    {
      // valeur correspondante aux deux prochains bits
      uint8_t value = (storage[itf][read_msg_i][read_byte_i] & and_op) >> read_offset;

      // on écrit cette valeur dans le message d'init_r (algorithme d'init_r)
      msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
      incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

      if (value == END_NODE)
	// si la valeur correspond à un fin de noeud
      {
	// on diminue la profondeur
	depth--;
      }
      else
	// sinon
      {
	// au contraire, on descend dans l'arbre donc on augmente la profondeur
	depth++;
      }
    }

    // on vide le stockage pour cette interface
    Empty_Storage(itf);
  }

  // on termine l'init_r avec un 'FIN DE NOEUD' (algorihtme init_r)
  msg_to_send[write_msg_i][write_byte_i] += END_NODE << write_offset;

  // le type du message : init_r
  uint8_t msg_type = INIT_R;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 6) + nb_msg;
    // on envoie le message au père
    Transmit(father_itf, msg_to_send[msg_i]);
  }

  state = "IR"; // TODO: a supprimer
}

void Module::Handle_Message_to_Module(uint8_t *pData)
{
  if (msg_to_store[father_itf] == 0)
    // si on ne sait pas encore combien de sous-messages on doit stocker
  {
    // comme les 6 derniers bits du header code le nombre de sous-messages
    // on regarde combien de sous-messages on doit stocker
    msg_to_store[father_itf] = (pData[0]) & 0b00111111;
  }

  // on stocke le message
  Store_Message(father_itf, pData);

  if (msg_stored[father_itf] == msg_to_store[father_itf])
    // si on a recu le bon nombre de sous-messages, on peut donc traiter la transmission
  {
    // segment routing : on regade les deux premiers bits du premier sous-message
    // pour savoir ce que l'on doit faire
    uint8_t next_itf = (storage[father_itf][0][1] & 0b11000000) >> 6;

    if (next_itf != END_SEGMENT_ROUTING)
      // si l'interface sur laquelle on doit transmettre le message
      // n'est pas un 'FIN DE SEGMENT ROUTING''
    {
      // on peut transférer le message à cette interface
      Transfer_Message_to_Module();
    }
    else
      // si c'est un 'FIN DE SEGMENT ROUTING', cela signifie que le message
      // est destiné à ce module
    {
      // on réécrit le message dans un unique tableau d'octets
      uint8_t message[(BUFFSIZE - 1) * NB_MAX_SBMSG] = {0};

      // indices de lecture et d'écriture
      int read_msg_i = 0;
      int read_byte_i = 1, write_byte_i = 0;
      int read_offset = 4, write_offset = 6;
      uint8_t and_op = 0b00110000;

      while (read_msg_i < msg_stored[father_itf])
	// tant que l'on n'a pas lu tous les sous-messages
      {
	// on réécrit dans le tableau du message
	uint8_t value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
	message[write_byte_i] += value << write_offset;
	incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

	// gestion des indices d'écriture
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

      // on peut maintenant lire le message
      Read_Message(message);

      // puis on vide le stockage de l'interface du père
      Empty_Storage(father_itf);
    }
  }
}

void Module::Transfer_Message_to_Module()
{
  // tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 1;
  int write_offset = 6;

  // indices de lecture
  int read_msg_i = 0;
  int read_byte_i = 1;
  int read_offset = 6;
  uint8_t and_op = 0b11000000;

  // segment routing : il faut transmettre le message au prochain itf codé sur les deux premiers
  // bits du message
  uint8_t next_itf = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
  incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

  uint8_t value = 0;
  while (value != END_SEGMENT_ROUTING)
    // on réécrit tant que l'on est pas arrivé à la fin du segment routing
  {
    // on réécrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);
  }

  // on lit la longueur du message en octet qui se situe juste après le segment routing
  // i.e. au tout début du message applicatif, sur le premier octet
  uint8_t length = 0;
  for (int bit_i = 0; bit_i < 8; bit_i += 2)
  {
    // on lit ce qu'on a stocke et ecrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);

    // on determine la longueur du message applicatif
    length += value << (6 - bit_i);
  }

  for (int bit_i = 0; bit_i < length*8; bit_i += 2)
    // on réécrit seulement ce qui nous interesse grâce à la longueur du message applicatif
    // (cela évite notamment d'envoyer un sous-message vide à la fin)
  {
    // on réécrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2);
  }

  // on vide le stockage de l'interface du père (l'interface par laquelle on recoit 
  // les messages)
  Empty_Storage(father_itf);

  // le type du message : message to son
  uint8_t msg_type = MSG_TO_MODULE;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 6) + nb_msg;
    // on envoie le message
    Transmit(next_itf, msg_to_send[msg_i]);
  }
}

void Module::Read_Message(uint8_t *pData)
{
  last_message = "";
  uint8_t length = pData[0];
  for (int byte_i = 1; byte_i < length + 1; byte_i++)
  {
    last_message.push_back((char)pData[byte_i]);
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
    message[i] = (uint8_t)text[i - 1];
  }
  Send_Message_to_Source(message, text_len + 1);
}

void Module::Handle_Message_to_Source(uint8_t *pData)
{
  // on doit simplement transmettre le message au père pour qu'il arrive jusqu'à la source
  Transmit(father_itf, pData);
}

void Module::Send_Message_to_Source(uint8_t *pData, uint8_t length)
{
  // tableau de stockage du message à envoyer
  uint8_t message[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i;
  int write_byte_i;

  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
    // pour chaque octet à envoyer
  {
    // on determine les bons indices d'écriture
    write_msg_i = read_byte_i / (BUFFSIZE - 1);
    write_byte_i = read_byte_i % (BUFFSIZE - 1) + write_msg_i + 1;

    // on recopie les données
    message[write_msg_i][write_byte_i] = pData[read_byte_i];
  }

  // le nombre de sous-messages
  int nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    message[msg_i][0] = (MSG_TO_SOURCE << 6) + nb_msg;
    // on envoie le message
    Transmit(father_itf, message[msg_i]);
  }
}

uint8_t Module::Transmit(uint8_t itf, uint8_t *pData)
{
  if (connections[itf] == NULL)
  {
    if (itf == father_itf)
    {
      if ((pData[0] & 0b11000000) >> 6 == INIT_R)
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
	  int value = ((pData[byte_i] & and_op) >> offset);
	  std::cout << value;
	  file << value;
	  incr_indexes(&msg_i, &byte_i, &offset, &and_op, 2);
	}
	std::cout << std::endl;
	file.close();
      }
      if ((pData[0] & 0b11000000) >> 6 == MSG_TO_SOURCE)
      {
	uint8_t length = pData[1];
	for (int byte_i = 2; byte_i < length + 2; byte_i++)
	{
	  std::cout << (char)pData[byte_i];
	}
	std::cout << std::endl;
      }
    }
    return 0;
  }
  else
  {
    Module *module_to_trsmt = connections[itf];
    // std::cout << "transmit (" << unsigned(id) << "): " << unsigned(itf) << std::endl;
    for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
    {
      module_to_trsmt->received[connections_other_side_itf[itf]][module_to_trsmt->received_nb[connections_other_side_itf[itf]]][byte_i] = pData[byte_i];

      /* std::bitset<8> byte (pData[byte_i]);
       * std::cout << byte << " ";
       * if (byte_i % 4 == 3)
       * {
       *   std::cout << std::endl;
       * }    */
    }

    module_to_trsmt->received_nb[connections_other_side_itf[itf]]++;
    return 1;
  }
}

void Module::Handle_All_Message()
{
  last_message = "no msg";
  for (int itf = 0; itf < NB_ITF; itf++)
  {
    for (int msg_i = 0; msg_i < received_nb[itf]; msg_i++)
    {
      Handle_Message(itf, received[itf][msg_i]);
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

void Module::set_father(Module *father, uint8_t itf_this_side, uint8_t itf_father_side)
{
  connections[itf_this_side] = father;
  connections_other_side_itf[itf_this_side] = itf_father_side;
}

void Module::set_first_son(Module *son, uint8_t itf_this_side, uint8_t itf_son_side)
{
  connections[itf_this_side] = son;
  connections_other_side_itf[itf_this_side] = itf_son_side;
  son_itfs_to_print[0] = itf_this_side;
}

void Module::set_second_son(Module *son, uint8_t itf_this_side, uint8_t itf_son_side)
{
  connections[itf_this_side] = son;
  connections_other_side_itf[itf_this_side] = itf_son_side;
  son_itfs_to_print[1] = itf_this_side;
}

void Module::print(int depth)
{
  std::string space(9 * depth, ' ');
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

  for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
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
