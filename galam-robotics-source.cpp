#include "galam-robotics-source.hpp"

int Transmit(uint8_t *pData, uint8_t length, uint16_t Timeout)
{
  // TODO : à compléter
  return 0;
}


int Send_init(uint8_t init_id)
{
  // le message d'init
  uint8_t init_msg[BUFFSIZE] = {0};
  // on ajoute le bon header
  init_msg[0] = (INIT << 5) + 1;
  // le ajoute l'id de l'init
  init_msg[1] = init_id;
  
  // on envoie
  return Transmit(init_msg, BUFFSIZE, TIME_OUT);
}


void Read_init_r(uint8_t *init_r, std::vector<uint8_t> *segment_routings)
{
  // indices de lecture
  int read_byte_i = 0;
  int read_offset = 6;
  uint8_t and_op = 0b11000000;

  // le vecteur du chemin actuel
  std::vector<uint8_t> path;
  // le numero de module actuel (le module 0 est le module connecté à la source)
  int module_i = 1;

  // on lit la première valeur
  int8_t value = (init_r[read_byte_i] & and_op) >> read_offset;
  // on met à jour les indices
  read_offset -= 2;
  and_op = and_op >> 2;

  while (path.size() > 0 || value != END_NODE)
    // tant que l'init_r n'a pas été lu entièrement
  {
    if (value == END_NODE)
      // si la valeur correspond à une fin de noeud
    {
      // on retire le dernier élément du chemin (on remonte dans l'arbre)
      path.pop_back();
    }
    else
      // sinon, c'est un nouveau module
    {
      // on ajoute l'interface au chemin (on descend dans l'arbre)
      path.push_back(value);
      // on enregistre le segment routing pour ce nouveau module
      segment_routings[module_i] = path;
      // on incrémente le numéro de module
      module_i++;
    }

    // on lit la prochaine valeur
    value = (init_r[read_byte_i] & and_op) >> read_offset;
    // on met à jour les indices
    if (read_offset == 0)
    {
      read_offset = 6;
      and_op = 0b11000000;
      read_byte_i++;
    }
    else
    {
      read_offset -= 2;
      and_op = and_op >> 2;
    }
  }
}


int Send_Message_Identification(uint16_t module_id, std::vector<uint8_t> segment_routing)
{
  // le tableau de stockage du message d'identification à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // les indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 1;
  int write_offset = 6;

  // on commence par écrire le segment routing dès le second octet du message
  for (uint8_t itf : segment_routing)
  {
    msg_to_send[write_msg_i][write_byte_i] += itf << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 1);
  }
  // on signale dans le message la fin du segment routing
  msg_to_send[write_msg_i][write_byte_i] += END_SEGMENT_ROUTING << write_offset;
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 1);

  // on écrit ensuite l'identifiant du module codé sur 2 octets
  uint16_t and_op = 0b1100000000000000;
  for (int read_offset = 14; read_offset >= 0; read_offset -= 2)
  {
    msg_to_send[write_msg_i][write_byte_i] += ((module_id & and_op) >> read_offset) << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 1);
    and_op = and_op >> 2;
  }

  // le type du message : identification message
  uint8_t msg_type = IDENTIFICATION;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  if (write_byte_i == 1 && write_offset == 6 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on envoie le message
    int t = Transmit(msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on renvoie 0
      return 0;
    }
  }

  return 1;
}


int Send_Message_to_Module(uint8_t *msg, uint8_t length, uint8_t msg_id, std::vector<uint8_t> segment_routing)
{
  // le tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // les indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 2;
  int write_offset = 6;

  // on commence par écrire le segment routing dès le troisième octet du message
  for (uint8_t itf : segment_routing)
  {
    msg_to_send[write_msg_i][write_byte_i] += itf << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
  }
  // on signale dans le message la fin du segment routing
  msg_to_send[write_msg_i][write_byte_i] += END_SEGMENT_ROUTING << write_offset;
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);

  // on écrit ensuite la longueur du message
  uint8_t and_op = 0b11000000;
  for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
  {
    msg_to_send[write_msg_i][write_byte_i] += ((length & and_op) >> read_offset) << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    and_op = and_op >> 2;
  }

  // puis enfin le contenu du message
  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
    // pour chaque octet
  {
    uint8_t and_op = 0b11000000;
    for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
      // on écrit l'octet dans le message, 2 bits par 2
    {
      msg_to_send[write_msg_i][write_byte_i] += ((msg[read_byte_i] & and_op) >> read_offset) << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
      and_op = and_op >> 2;
    }
  }

  // le type du message : message to module
  uint8_t msg_type = MSG_TO_MODULE;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  if (write_byte_i == 2 && write_offset == 6 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du message
    msg_to_send[msg_i][1] = msg_id;
    // on envoie le message
    int t = Transmit(msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on renvoie 0
      return 0;
    }
  }

  return 1;
}


int Send_Message_to_Multiple_Modules(uint8_t *msg, uint8_t length, uint8_t msg_id, std::vector<std::vector<uint8_t>> segment_routings)
{
  // on copie le vecteur des segment routings
  std::vector<std::vector<uint8_t>> seg_rts (segment_routings);

  // le tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // les indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 2;
  int write_offset = 6;

  // on trie les segment routings
  std::sort (seg_rts.begin(), seg_rts.end(), sort_segment_routings);

  // on écrit si le premier module doit lire le message ou non
  if (seg_rts[0].size() == 0)
    // si le premier segment routing est vide, c'est donc celui du premier module
  {
    // il doit alors lire le message
    msg_to_send[write_msg_i][write_byte_i] += (1 << write_offset);
    seg_rts.erase(seg_rts.begin());
  }
  else
  {
    // sinon il ne le lit pas
    msg_to_send[write_msg_i][write_byte_i] += (0 << write_offset);
  }
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
  
  // le vecteur du chemin actuel
  std::vector<uint8_t> path;
  while (seg_rts.size() != 0 || path.size() != 0)
    // tant qu'il reste des segment routings ou que le chemin actuel n'a pas été vidé
  {
    // on regarde si le chemin actuel débute de la même façon que le prochain segment routing
    bool same_path = true;
    if (seg_rts.size() > 0)
      // si il reste des segment routings
    {
      if (path.size() <= seg_rts[0].size())
	// si la taille du chemin actuel est bien plus petite que le prochain segment routing
      {
	// on compare
	for (int i = 0; i < path.size(); i++)
	{
	  if (path[i] != seg_rts[0][i])
	  {
	    same_path = false;
	    break;
	  }
	}
      }
      else {same_path = false;}
    }

    if (!same_path || seg_rts.size() == 0)
      // si les débuts de chemins ne sont pas identique ou si il n'y a plus de segment routings à
      // traiter
    {
      // on écrit un FIN DE NOEUD (on remonte dans l'arbre)
      msg_to_send[write_msg_i][write_byte_i] += ((END_NODE & 0b00000010) >> 1) << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
      msg_to_send[write_msg_i][write_byte_i] += (END_NODE & 0b00000001) << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);

      // on enlève le dernier élement du chemin
      path.erase(path.end()-1);
    }
    else
      // sinon, on descend dans l'arbre
    {
      // on écrit la suite du segment routing, à partir de la fin du chemin
      for (int i = path.size(); i < seg_rts[0].size(); i++)
      {
	msg_to_send[write_msg_i][write_byte_i] += ((seg_rts[0][i] & 0b00000010) >> 1) << write_offset;
	incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
	msg_to_send[write_msg_i][write_byte_i] += (seg_rts[0][i] & 0b00000001) << write_offset;
	incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);

	// on ajoute l'élement au chemin
	path.push_back(seg_rts[0][i]);
	
	if (i != seg_rts[0].size() - 1)
	  // si on se situe à un module dans le chemin
	{
	  // ce module ne doit pas lire le message
	  msg_to_send[write_msg_i][write_byte_i] += 0 << write_offset;
	}
	else
	  // si on a atteint le module en question
	{
	  // lui doit lire le message
	  msg_to_send[write_msg_i][write_byte_i] += 1 << write_offset;
	}
	incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
      }

      // on passe au prochain segment routing
      seg_rts.erase(seg_rts.begin());
    }
  }
  // on termine le segment routing avec une FIN DE NOEUD
  msg_to_send[write_msg_i][write_byte_i] += ((END_NODE & 0b00000010) >> 1) << write_offset;
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
  msg_to_send[write_msg_i][write_byte_i] += (END_NODE & 0b00000001) << write_offset;
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);

  // on écrit ensuite la longueur du message
  uint8_t and_op = 0b10000000;
  for (int read_offset = 7; read_offset >= 0; read_offset--)
  {
    msg_to_send[write_msg_i][write_byte_i] += ((length & and_op) >> read_offset) << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
    and_op = and_op >> 1;
  }

  // puis enfin le contenu du message
  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
    // pour chaque octet
  {
    uint8_t and_op = 0b10000000;
    for (int read_offset = 7; read_offset >= 0; read_offset--)
      // on écrit l'octet dans le message, 2 bits par 2
    {
      msg_to_send[write_msg_i][write_byte_i] += ((msg[read_byte_i] & and_op) >> read_offset) << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 1, 2);
      and_op = and_op >> 1;
    }
  }

  // le type du message : message to multiple modules
  uint8_t msg_type = MSG_TO_MULT_MODULES;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  if (write_byte_i == 2 && write_offset == 6 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du message
    msg_to_send[msg_i][1] = msg_id;
    // on envoie le message
    int t = Transmit(msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on renvoie 0
      return 0;
    }
  }

  return 1;
}


int Send_Message_to_All(uint8_t *msg, uint8_t length, uint8_t msg_id)
{
  // le tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // les indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 2;

  // on écrit la longueur du message
  msg_to_send[write_msg_i][write_byte_i] = length;
  write_byte_i++;

  // puis le contenu du message
  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
    // pour chaque octet
  {
    // on recopie l'octet
    msg_to_send[write_msg_i][write_byte_i] = msg[read_byte_i];

    // on met à jour les indices
    if (write_byte_i == BUFFSIZE - 1)
    {
      write_byte_i = 2;
      write_msg_i++;
    }
    else
    {
      write_byte_i++;
    }
  }

  // le type du message : message to all
  uint8_t msg_type = MSG_TO_ALL;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  if (write_byte_i == 2 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du message
    msg_to_send[msg_i][1] = msg_id;
    // on envoie le message
    int t = Transmit(msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on renvoie 0
      return 0;
    }
  }

  return 1;
}


void incr_indexes(int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr, int byte_start)
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
      *byte_i = byte_start;
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


bool sort_segment_routings (std::vector<uint8_t> seg_rt1, std::vector<uint8_t> seg_rt2)
{
  // on prend la taille la plus petite des 2
  int size = seg_rt1.size();
  if (seg_rt2.size() < size) {size = seg_rt2.size();}

  for (int i = 0; i < size; i++)
  {
    // on compare
    if (seg_rt1[i] < seg_rt2[i]) {return true;}
    else if(seg_rt2[i] < seg_rt1[i]) {return false;}
  }
  // si égaux, c'est la taille qui prévaut
  return seg_rt1.size() < seg_rt2.size();
}
