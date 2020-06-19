#include "galam-robotics-module.h"

// --- VARIABLES ---

uint8_t rx0[BUFFSIZE]; // tableaux pour réceptionner les messages
uint8_t rx1[BUFFSIZE];
uint8_t rx2[BUFFSIZE];

uint8_t father_itf = UNKNOWN_ITF; // interface du père
uint8_t son_itfs[2] = {UNKNOWN_ITF, UNKNOWN_ITF}; // identifiants d'interface des fils
int son_nb = 0; // nombre de fils
bool init_r_sent = false; // booléen qui indique si l'init_r a été envoyé

uint8_t msg_stored[3] = {0, 0, 0}; // nombre de sous-messages stockés par interface
uint8_t msg_to_store[3] = {0, 0, 0}; // nombre de sous-message à stocker par interface
uint8_t storage[NB_ITF][NB_MAX_SBMSG][BUFFSIZE] = {0}; // tableau de stockage des sous-messages

uint16_t id = UNKNOWN_ID;  // id du module (modifiable grâce au message de type "identification")

// --- FONCTIONS ---

void init()
{
  Receive_IT(0, rx0, BUFFSIZE);
  Receive_IT(1, rx1, BUFFSIZE);
  Receive_IT(2, rx2, BUFFSIZE);
}

int main() {
  while (1 == 1)
  {
    // TODO : à compléter
  }
}

// ======================================= COUCHE RÉSEAU ============================================

int Receive_IT(uint8_t itf, uint8_t *pData, uint8_t length)
{
  // TODO : à compléter
  return 0;
}


int RxCallBack(uint8_t itf)
{
  if(itf == 0)
  {
    Handle_Message(itf, rx0);
    Receive_IT(itf, rx0, BUFFSIZE);
  }

  if(itf == 1)
  {
    Handle_Message(itf, rx1);
    Receive_IT(itf, rx1, BUFFSIZE);
  }

  if(itf == 2)
  {
    Handle_Message(itf, rx2);
    Receive_IT(itf, rx2, BUFFSIZE);
  }
  return 0;
}


int Transmit(uint8_t itf, uint8_t *pData, uint8_t length, uint16_t Timeout)
{
  // TODO : à compléter
  return 0;
}


void Handle_Message(uint8_t itf, uint8_t *pData)
{
  // le type du message est écrit sur les trois premiers bits du header
  // on recupère donc ce numéro codant ce type
  uint8_t msg_type = ((pData[0]) & 0b11100000) >> 5;

  // on redirige vers la bonne fonction
  if (msg_type == INIT) // init
  {
    Handle_Message_init(itf, pData);
  }
  else if (msg_type == INIT_R) // init_r
  {
    Handle_Message_init_r(itf, pData);
  }
  else if (msg_type == IDENTIFICATION) // identification message
  {
    Handle_Message_Identification(pData);
  }
  else if (msg_type == MSG_TO_MODULE) // message to module
  {
    Handle_Message_to_Module(pData);
  }
  else if (msg_type == MSG_TO_SOURCE) // message to source
  {
    Handle_Message_to_Source(pData);
  }
  else if(msg_type == MSG_TO_MULT_MODULES) // message to multiple modules
  {
    Handle_Message_to_Multiple_Modules(pData);
  }
  else if (msg_type == MSG_TO_ALL) // message to all
  {
    Handle_Message_to_All(pData);
  }
  else
    // on prévient la source qu'on a reçu un message de type inconnu
  {
    // le contenu du message
    char* msg_content = "reception d'un message de type inconnu";

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_UNKNOWN_MSG_TYPE, error_msg, length);
  }
}


void Handle_Message_init(uint8_t itf, uint8_t *pData)
{
  if (father_itf != UNKNOWN_ITF && itf != father_itf && !init_r_sent)
    // si on a déjà reçu l'init mais qu'on le reçoit à nouveau depuis une interface qui n'est pas celle
    // du père et que l'init_r n'a pas encore été envoyé, cela signifie qu'il y a une boucle dans l'arbre
    // des modules
  {
    // on prévient alors la source de ce problème

    // TODO : ignorer l'interface + ajouter un id à l'init
    // le contenu du message
    char* error_text = "boucle de modules avec l'interface ITF";
    char error_itf[2];
    sprintf(error_itf, "%d", itf);
    char* msg_content = malloc(strlen(error_text)+2);
    strcpy(msg_content, error_text);
    strcat(msg_content, error_itf);

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_MODULES_LOOP, error_msg, length);
  }
  else
    // sinon, on peut initialiser ou réinitialiser le module
  {
    // on reset toutes les variables (utile dans le cas du réinitialisation des modules)
    for (int i = 0; i < NB_ITF - 1; i++) {son_itfs[i] = UNKNOWN_ITF;}
    son_nb = 0;
    init_r_sent = false;
    for (int interface = 0; interface < NB_ITF; interface++) {Empty_Storage(interface);}
    id = 0;

    // comme le message init provient toujours de la source, l'interface depuis laquelle on recoit
    // l'init est donc l'interface du père du module (dans l'arbre)
    father_itf = itf;


    if (((pData[0]) & 0b00000111) == 1)
      // les 3 derniers bits de l'header code le nombre de sous-messages
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
	  uint8_t t = Transmit(itf, pData, BUFFSIZE, TIME_OUT);

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

    if (son_nb == 0)
      // si le noeud n'a pas de fils, on envoie tout de suite l'init_r
    {
      Send_init_r();
    }
  }
}


void Handle_Message_init_r(uint8_t itf, uint8_t *pData)
{
  // les 5 derniers bits du header code le nombre de sous-messages
  // ainsi, on regarde combien de sous-messages on doit stocker
  if (msg_to_store[itf] == 1)
  {
    msg_to_store[itf] = (pData[0]) & 0b00000111;
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


int Send_init_r()
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
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 1);

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
      if (read_msg_i == msg_stored[itf])
	// si on dépasse la fin du message
      {
	// on prévient alors la source de ce problème

	// le contenu du message
	char* error_text = "init_r illisible depuis l'interface ITF";
	char error_itf[2];
	sprintf(error_itf, "%d", itf);
	char* msg_content = malloc(strlen(error_text)+2);
	strcpy(msg_content, error_text);
	strcat(msg_content, error_itf);

	// longueur du message
	const uint8_t length = strlen(msg_content);

	// le message d'erreur
	uint8_t error_msg[length];
	// on remplit avec le contenu du message
	for (int i = 0; i < length; i++)
	{
	  error_msg[i] = (uint8_t) msg_content[i];
	}
	// on envoie le message d'erreur à la source
	Send_Error_Message_to_Source(E_NET_UNREADABLE_INIT_R, error_msg, length);
	return 0;
      }

      // valeur correspondante aux deux prochains bits
      uint8_t value = (storage[itf][read_msg_i][read_byte_i] & and_op) >> read_offset;

      // on écrit cette valeur dans le message d'init_r (algorithme d'init_r)
      msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 1);
      incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 1);

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
  if (write_byte_i == 1 && write_offset == 6 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= nb_msg - 1; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on envoie le message au père
    Transmit(father_itf, msg_to_send[msg_i], BUFFSIZE, TIME_OUT);
  }

  // on actualise cette variable
  init_r_sent = true;
  return 1;
}


void Handle_Message_Identification(uint8_t *pData)
{
  // les 2 bits après le type de message code le numéro de ce sous-message
  uint8_t sub_msg_number = (pData[0] & 0b00011000) >> 3;
  // les 3 derniers bits du header code le nombre total de sous-messages
  uint8_t sub_msg_total = (pData[0]) & 0b00000111;

  if (msg_to_store[father_itf] == 0 && sub_msg_number == 0)
    // si ce sous-message est le premier
  {
    // on regarde combien de sous-messages on doit stocker
    msg_to_store[father_itf] = sub_msg_total;
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else if (msg_stored[father_itf] == sub_msg_number)
    // si c'est la suite d'un message
  {
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else
    // si il manque un sous-message
  {
    // on signale ce problème à la source

    // contenu du message d'erreur
    char* error_text0 = "reception du message ID";
    char error_msg_id[4];
    sprintf(error_msg_id, "%d", storage[father_itf][0][1]);
    char * error_text1 = " incomplete";
    char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
    strcpy(msg_content, error_text0);
    strcat(msg_content, error_msg_id);
    strcat(msg_content, error_text1);

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_UNCOMPLETE_MSG, error_msg, length);
  }

  if (msg_stored[father_itf] == msg_to_store[father_itf])
    // si on a recu le bon nombre de sous-messages, on peut donc traiter la transmission
  {
    // segment routing : on regade les deux premiers bits du premier sous-message
    // pour savoir ce que l'on doit faire
    uint8_t next_itf = (storage[father_itf][0][2] & 0b11000000) >> 6;

    if (next_itf != END_SEGMENT_ROUTING)
      // si l'interface sur laquelle on doit transmettre le message
      // n'est pas un 'FIN DE SEGMENT ROUTING''
    {
      // on peut transférer le message à cette interface
      Transmit_Message_Identification();
    }
    else
      // si c'est un 'FIN DE SEGMENT ROUTING', cela signifie que le message d'identification
      // est destiné à ce module
    {
      // indices de lecture et d'écriture
      int read_byte_i = 2;
      int read_offset = 4;
      uint8_t and_op = 0b00110000;

      // on reinitialise l'identiant du module
      id = 0;
      for (int bit_i = 0; bit_i < 16; bit_i += 2)
	// on lit l'identifiant codé sur 2 octets
      {
	uint8_t value = (storage[father_itf][0][read_byte_i] && and_op) >> read_offset;
	id += value << (14 - bit_i);

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

      // on vide le stockage de l'interface du père
      Empty_Storage(father_itf);
    }
  }
}


void Transmit_Message_Identification()
{
  // l'id du message situé sur le second octet
  uint8_t msg_id = storage[father_itf][0][1];

  // tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 2;
  int write_offset = 6;

  // indices de lecture
  int read_msg_i = 0;
  int read_byte_i = 2;
  int read_offset = 6;
  uint8_t and_op = 0b11000000;

  // segment routing : il faut transmettre le message au prochain itf codé sur les deux premiers
  // bits du message
  uint8_t next_itf = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
  incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);

  uint8_t value = 0;
  while (value != END_SEGMENT_ROUTING)
    // on réécrit tant que l'on est pas arrivé à la fin du segment routing
  {
    // on réécrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);
  }

  // on recopie l'identifiant codé sur 2 octets après le segment routing
  for (int bit_i = 0; bit_i < 16; bit_i += 2)
  {
    // on lit ce qu'on a stocké et ecrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);
  }

  // on vide le stockage de l'interface du père (l'interface par laquelle on recoit
  // les messages)
  Empty_Storage(father_itf);

  // le type du message : identification message
  uint8_t msg_type = IDENTIFICATION;
  // le nombre de sous-messages pour cette transmission
  uint8_t nb_msg = write_msg_i + 1;
  if (write_byte_i == 2 && write_offset == 6 && nb_msg > 1)
    // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
    // sous-message
  {
    nb_msg--;
  }
  for (int msg_i = 0; msg_i <= nb_msg - 1; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du message
    msg_to_send[msg_i][1] = msg_id;
    // on envoie le message
    uint8_t t = Transmit(next_itf, msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on signale ce problème à la source

      // contenu du message d'erreur
      char* error_text = "echec de transmission du message d'identification vers l'interface ITF";
      char error_itf[2];
      sprintf(error_itf, "%d", next_itf);
      char* msg_content = malloc(strlen(error_text)+2);
      strcpy(msg_content, error_text);
      strcat(msg_content, error_itf);

      // longueur du message
      const uint8_t length = strlen(msg_content);

      // le message d'erreur
      uint8_t error_msg[length];
      // on remplit avec le contenu du message
      for (int i = 0; i < length; i++)
      {
	error_msg[i] = (uint8_t) msg_content[i];
      }
      // on envoie le message d'erreur à la source
      Send_Error_Message_to_Source(E_NET_CANT_TRSMT_ID, error_msg, length);
      break;
    }
  }
}


void Handle_Message_to_Module(uint8_t *pData)
{
  // les 2 bits après le type de message code le numéro de ce sous-message
  uint8_t sub_msg_number = (pData[0] & 0b00011000) >> 3;
  // les 3 derniers bits du header code le nombre total de sous-messages
  uint8_t sub_msg_total = (pData[0]) & 0b00000111;

  if (msg_to_store[father_itf] == 0 && sub_msg_number == 0)
    // si ce sous-message est le premier
  {
    // on regarde combien de sous-messages on doit stocker
    msg_to_store[father_itf] = sub_msg_total;
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else if (msg_stored[father_itf] == sub_msg_number)
    // si c'est la suite d'un message
  {
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else
    // si il manque un sous-message
  {
    // on signale ce problème à la source

    // contenu du message d'erreur
    char* error_text0 = "reception du message ID";
    char error_msg_id[4];
    sprintf(error_msg_id, "%d", storage[father_itf][0][1]);
    char * error_text1 = " incomplete";
    char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
    strcpy(msg_content, error_text0);
    strcat(msg_content, error_msg_id);
    strcat(msg_content, error_text1);

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_UNCOMPLETE_MSG, error_msg, length);

    // on vide le stockage de l'interface du père pour être prêt à recevoir un nouveau message
    Empty_Storage(father_itf);

    if (sub_msg_number == 0)
      // c'est un nouveau message
    {
      // on regarde combien de sous-messages on doit stocker
      msg_to_store[father_itf] = sub_msg_total;
      // on stocke le message
      Store_Message(father_itf, pData);
    }
  }

  if (msg_stored[father_itf] == msg_to_store[father_itf])
    // si on a recu le bon nombre de sous-messages, on peut donc traiter la transmission
  {
    // segment routing : on regade les deux premiers bits du premier sous-message
    // pour savoir ce que l'on doit faire
    uint8_t next_itf = (storage[father_itf][0][2] & 0b11000000) >> 6;

    if (next_itf != END_SEGMENT_ROUTING)
      // si l'interface sur laquelle on doit transmettre le message
      // n'est pas un 'FIN DE SEGMENT ROUTING''
    {
      // on peut transférer le message à cette interface
      Transmit_Message_to_Module();
    }
    else
      // si c'est un 'FIN DE SEGMENT ROUTING', cela signifie que le message
      // est destiné à ce module
    {
      // on récupère l'id du message situé sur le second octet
      uint8_t msg_id = storage[father_itf][0][1];

      // on réécrit le message dans un unique tableau d'octets
      uint8_t message[(BUFFSIZE - 2) * NB_MAX_SBMSG] = {0};

      // indices de lecture et d'écriture
      int read_msg_i = 0;
      int read_byte_i = 2, write_byte_i = 0;
      int read_offset = 4, write_offset = 6;
      uint8_t and_op = 0b00110000;

      uint8_t length = 0;
      for (int bit_i = 0; bit_i < 8; bit_i += 2)
	// on lit la longueur du message
      {
	uint8_t value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
	incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);
	length += value << (6 - bit_i);
      }

      bool error = false;
      for (int bit_i = 0; bit_i < length * 8; bit_i += 2)
	// tant que l'on n'a pas lu le message en entier
      {
	if (read_msg_i == msg_stored[father_itf])
	  // si on dépasse la fin du message
	{
	  // on prévient alors la source de ce problème

	  // le contenu du message
	  char* error_text0 = "message ID";
	  char error_msg_id[4];
	  sprintf(error_msg_id, "%d", msg_id);
	  char* error_text1 = " illisible";
	  char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
	  strcpy(msg_content, error_text0);
	  strcat(msg_content, error_msg_id);
	  strcat(msg_content, error_text1);

	  // longueur du message
	  const uint8_t length = strlen(msg_content);

	  // le message d'erreur
	  uint8_t error_msg[length];
	  // on remplit avec le contenu du message
	  for (int i = 0; i < length; i++)
	  {
	    error_msg[i] = (uint8_t) msg_content[i];
	  }
	  // on envoie le message d'erreur à la source
	  Send_Error_Message_to_Source(E_NET_UNREADABLE_MSG, error_msg, length);
	  error = true;
	  break;
	}

	// on réécrit dans le tableau du message
	uint8_t value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
	message[write_byte_i] += value << write_offset;
	incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);

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

      if (!error)
      {
	// on peut maintenant lire le message
	Read_Message(msg_id, message, length);
      }

      // puis on vide le stockage de l'interface du père
      Empty_Storage(father_itf);
    }
  }
}


void Transmit_Message_to_Module()
{
  // l'id du message situé sur le second octet
  uint8_t msg_id = storage[father_itf][0][1];

  // tableau de stockage du message à envoyer
  uint8_t msg_to_send[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i = 0;
  int write_byte_i = 2;
  int write_offset = 6;

  // indices de lecture
  int read_msg_i = 0;
  int read_byte_i = 2;
  int read_offset = 6;
  uint8_t and_op = 0b11000000;

  // segment routing : il faut transmettre le message au prochain itf codé sur les deux premiers
  // bits du segment routing
  uint8_t next_itf = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
  incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);

  uint8_t value = 0;
  while (value != END_SEGMENT_ROUTING)
    // on réécrit tant que l'on est pas arrivé à la fin du segment routing
  {
    // on réécrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);
  }

  // on lit la longueur dVu message en octet qui se situe juste après le segment routing
  // i.e. au tout début du message applicatif, sur le premier octet
  uint8_t length = 0;
  for (int bit_i = 0; bit_i < 8; bit_i += 2)
  {
    // on lit ce qu'on a stocké et ecrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);

    // on determine la longueur du message applicatif
    length += value << (6 - bit_i);
  }

  for (int bit_i = 0; bit_i < length * 8; bit_i += 2)
    // on réécrit seulement ce qui nous interesse grâce à la longueur du message applicatif
    // (cela évite notamment d'envoyer un sous-message vide à la fin)
  {
    if (read_msg_i == msg_stored[father_itf])
      // si on atteint le nombre maximum de sous-messages, on arrête
    {
      break;
    }

    // on réécrit dans le tableau du message
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    msg_to_send[write_msg_i][write_byte_i] += value << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2, 2);
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 2, 2);
  }

  // on vide le stockage de l'interface du père (l'interface par laquelle on recoit
  // les messages)
  Empty_Storage(father_itf);

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
  for (int msg_i = 0; msg_i <= nb_msg - 1; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    msg_to_send[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du message
    msg_to_send[msg_i][1] = msg_id;
    // on envoie le message
    uint8_t t = Transmit(next_itf, msg_to_send[msg_i], BUFFSIZE, TIME_OUT);

    if (t == 0)
      // si la transmission a échoué
    {
      // on signale ce problème à la source

      // contenu du message d'erreur
      char* error_text0 = "echec de transmission du message ID";
      char error_msg_id[4];
      sprintf(error_msg_id, "%d", msg_id);
      char * error_text1 = " vers l'interface ITF";
      char error_itf[2];
      sprintf(error_itf, "%d", next_itf);
      char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1)+2);
      strcpy(msg_content, error_text0);
      strcat(msg_content, error_msg_id);
      strcat(msg_content, error_text1);
      strcat(msg_content, error_itf);

      // longueur du message
      const uint8_t length = strlen(msg_content);

      // le message d'erreur
      uint8_t error_msg[length];
      // on remplit avec le contenu du message
      for (int i = 0; i < length; i++)
      {
	error_msg[i] = (uint8_t) msg_content[i];
      }
      // on envoie le message d'erreur à la source
      Send_Error_Message_to_Source(E_NET_CANT_TRSMT_MSG, error_msg, length);
      break;
    }
  }
}


void Handle_Message_to_Source(uint8_t *pData)
{
  // on doit simplement transmettre le message au père pour qu'il arrive jusqu'à la source
  Transmit(father_itf, pData, BUFFSIZE, TIME_OUT);
}


void Send_Message_to_Source(uint8_t *pData, uint8_t length)
{
  // tableau de stockage du message à envoyer
  uint8_t message[NB_MAX_SBMSG][BUFFSIZE] = {0};

  // longueur du message
  message[0][3] = length;

  // indices d'écriture
  int write_msg_i;
  int write_byte_i;

  for (int read_byte_i = 0; read_byte_i < length; read_byte_i++)
    // pour chaque octet à envoyer
  {
    // on determine les bons indices d'écriture
    write_msg_i = read_byte_i / (BUFFSIZE - 4);
    write_byte_i = read_byte_i % (BUFFSIZE - 4) + 3 + (write_msg_i == 0);

    // on recopie les données
    message[write_msg_i][write_byte_i] = pData[read_byte_i];
  }

  // le type du message : message to source
  uint8_t msg_type = MSG_TO_SOURCE;
  // le nombre de sous-messages
  int nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i; msg_i++)
    // pour chaque sous-message
  {
    // on ajoute le bon header
    message[msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
    // on ajoute l'id du module sur les deux octets suivant
    message[msg_i][1] = (id & 0b1111111100000000) >> 8;
    message[msg_i][2] = id & 0b0000000011111111;
    // on envoie le message
    Transmit(father_itf, message[msg_i], BUFFSIZE, TIME_OUT);
  }
}


void Handle_Message_to_Multiple_Modules(uint8_t *pData)
{
  // les 2 bits après le type de message code le numéro de ce sous-message
  uint8_t sub_msg_number = (pData[0] & 0b00011000) >> 3;
  // les 3 derniers bits du header code le nombre total de sous-messages
  uint8_t sub_msg_total = (pData[0]) & 0b00000111;

  if (msg_to_store[father_itf] == 0 && sub_msg_number == 0)
    // si ce sous-message est le premier
  {
    // on regarde combien de sous-messages on doit stocker
    msg_to_store[father_itf] = sub_msg_total;
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else if (msg_stored[father_itf] == sub_msg_number)
    // si c'est la suite d'un message
  {
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else
    // si il manque un sous-message
  {
    // on signale ce problème à la source

    // contenu du message d'erreur
    char* error_text0 = "reception du message ID";
    char error_msg_id[4];
    sprintf(error_msg_id, "%d", storage[father_itf][0][1]);
    char * error_text1 = " incomplete";
    char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
    strcpy(msg_content, error_text0);
    strcat(msg_content, error_msg_id);
    strcat(msg_content, error_text1);

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_UNCOMPLETE_MSG, error_msg, length);

    // on vide le stockage de l'interface du père pour être prêt à recevoir un nouveau message
    Empty_Storage(father_itf);

    if (sub_msg_number == 0)
      // c'est un nouveau message
    {
      // on regarde combien de sous-messages on doit stocker
      msg_to_store[father_itf] = sub_msg_total;
      // on stocke le message
      Store_Message(father_itf, pData);
    }
  }

  if (msg_stored[father_itf] == msg_to_store[father_itf])
    // si on a recu le bon nombre de sous-messages, on peut donc traiter la transmission
  {
    // on transfère le message aux fils
    int end_segment_routing = Transmit_Message_to_Multiple_Modules();

    if (end_segment_routing != 0)
      // si la lecture du segment routing n'a pas echoué
    {
      // on récupère l'id du message situé sur le second octet
      uint8_t msg_id = storage[father_itf][0][1];

      // segment routing ++ : le premier bit du segement routing nous indique si on ce module doit
      // lire le message
      uint8_t read = (storage[father_itf][0][2] & 0b10000000) >> 7;

      if (read == 1)
	// si ce module doit lire le message
      {
	// on réécrit le message dans un unique tableau d'octets
	uint8_t message[(BUFFSIZE - 2) * NB_MAX_SBMSG] = {0};

	// indices de lecture que l'on déduit grâce au bits où se trouve la fin du segment routing ++
	int read_msg_i = end_segment_routing / (8 * (BUFFSIZE - 2));
	end_segment_routing = end_segment_routing % (8 * (BUFFSIZE - 2));
	int read_byte_i = 2 + end_segment_routing / 8;
	int read_offset = 7 - (end_segment_routing % 8);
	uint8_t and_op = 0b10000000 >> (end_segment_routing % 8);

	// indices d'écriture
	int write_byte_i = 0;
	int write_offset = 7;

	uint8_t length = 0;
	for (int bit_i = 0; bit_i < 8; bit_i++)
	  // on lit la longueur du message
	{
	  uint8_t value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
	  incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
	  length += value << (7 - bit_i);
	}

	bool error = false;
	for (int bit_i = 0; bit_i < length * 8; bit_i++)
	  // tant que l'on n'a pas lu le message en entier
	{
	  if (read_msg_i == msg_stored[father_itf])
	    // si on dépasse la fin du message
	  {
	    // on prévient alors la source de ce problème

	    // le contenu du message
	    char* error_text0 = "message ID";
	    char error_msg_id[4];
	    sprintf(error_msg_id, "%d", msg_id);
	    char* error_text1 = " illisible";
	    char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
	    strcpy(msg_content, error_text0);
	    strcat(msg_content, error_msg_id);
	    strcat(msg_content, error_text1);

	    // longueur du message
	    const uint8_t length = strlen(msg_content);

	    // le message d'erreur
	    uint8_t error_msg[length];
	    // on remplit avec le contenu du message
	    for (int i = 0; i < length; i++)
	    {
	      error_msg[i] = (uint8_t) msg_content[i];
	    }
	    // on envoie le message d'erreur à la source
	    Send_Error_Message_to_Source(E_NET_UNREADABLE_MSG, error_msg, length);
	    error = true;
	    break;
	  }
	  // on réécrit dans le tableau du message
	  uint8_t value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
	  message[write_byte_i] += value << write_offset;
	  incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);

	  // gestion des indices d'écriture
	  if (write_offset == 0)
	  {
	    write_offset = 7;
	    write_byte_i++;
	  }
	  else
	  {
	    write_offset--;
	  }
	}

	if (!error)
	{
	  // on peut maintenant lire le message
	  Read_Message(msg_id, message, length);
	}
      }

      // puis on vide le stockage de l'interface du père
      Empty_Storage(father_itf);
    }
  }
}


int Transmit_Message_to_Multiple_Modules()
{
  // l'id du message situé sur le second octet
  uint8_t msg_id = storage[father_itf][0][1];

  // tableaux de stockage des messages à envoyer aux fils
  uint8_t msg_to_send[NB_ITF - 1][NB_MAX_SBMSG][BUFFSIZE] = {0};

  // indices d'écriture
  int write_msg_i[NB_ITF - 1] = {0};
  int write_byte_i[NB_ITF - 1];
  for (int i = 0; i < NB_ITF - 1; i++) {write_byte_i[i] = 2;}
  int write_offset[NB_ITF - 1];
  for (int i = 0; i < NB_ITF - 1; i++) {write_offset[i] = 7;}

  // indices de lecture
  int read_msg_i = 0;
  int read_byte_i = 2;
  int read_offset = 7;
  uint8_t and_op = 0b10000000;

  // tableau qui contiendra les interfaces par lesquelles on doit transmettre le message
  uint8_t itfs_to_trsmt[NB_ITF - 1] = {UNKNOWN_ITF};
  int nb_itfs_to_trsmt = 0;

  // compteur de bits avant la fin du segment routing ++
  int end_segment_routing = 0;
  // profondeur dans le arbre depuis ce module dans le segment routing ++
  int depth = 0;

  uint8_t itf = 0;
  uint8_t value;
  while (depth >= 0)
  {
    if (read_msg_i == msg_stored[father_itf])
      // si on dépasse la fin du message
    {
      // on prévient alors la source de ce problème

      // le contenu du message
      char* error_text = "segment routing illisible dans le message ID";
      char error_msg_id[4];
      sprintf(error_msg_id, "%d", msg_id);
      char* msg_content = malloc(strlen(error_text)+4);
      strcpy(msg_content, error_text);
      strcat(msg_content, error_msg_id);

      // longueur du message
      const uint8_t length = strlen(msg_content);

      // le message d'erreur
      uint8_t error_msg[length];
      // on remplit avec le contenu du message
      for (int i = 0; i < length; i++)
      {
	error_msg[i] = (uint8_t) msg_content[i];
      }
      // on envoie le message d'erreur à la source
      Send_Error_Message_to_Source(E_NET_UNREADABLE_SEG_RT, error_msg, length);
      return 0;
    }

    if (itf != END_NODE)
      // si la dernière interface lue n'est pas une fin de noeud
    {
      // bit de lecture
      value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
      incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
      if (depth > 0)
	// si on est dans une partie de segment routing a transmettre à un fils
      {
	// on le recopie dans le message à envoyer
	msg_to_send[nb_itfs_to_trsmt - 1][write_msg_i[nb_itfs_to_trsmt - 1]][write_byte_i[
	  nb_itfs_to_trsmt - 1]] += value << write_offset[nb_itfs_to_trsmt - 1];
	incr_indexes(&write_msg_i[nb_itfs_to_trsmt - 1], &write_byte_i[nb_itfs_to_trsmt - 1],
	    &write_offset[nb_itfs_to_trsmt - 1], NULL, 1, 2);
      }

      // on incremente le compteur de bits
      end_segment_routing++;
    }

    itf = 0;
    // premier bit d'itf
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
    itf += value << 1;
    if (depth > 0)
      // si on est dans une partie de segment routing a transmettre à un fils
    {
      // on le recopie dans le message à envoyer
      msg_to_send[nb_itfs_to_trsmt - 1][write_msg_i[nb_itfs_to_trsmt - 1]][write_byte_i[
	nb_itfs_to_trsmt - 1]] += value << write_offset[nb_itfs_to_trsmt - 1];
      incr_indexes(&write_msg_i[nb_itfs_to_trsmt - 1], &write_byte_i[nb_itfs_to_trsmt - 1],
	  &write_offset[nb_itfs_to_trsmt - 1], NULL, 1, 2);
    }
    // on incremente le compteur de bits
    end_segment_routing++;

    // second bit d'itf
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
    itf += value;
    if (depth > 0)
      // si on est dans une partie de segment routing a transmettre à un fils
    {
      // on le recopie dans le message à envoyer
      msg_to_send[nb_itfs_to_trsmt - 1][write_msg_i[nb_itfs_to_trsmt - 1]][write_byte_i[
	nb_itfs_to_trsmt - 1]] += value << write_offset[nb_itfs_to_trsmt - 1];
      incr_indexes(&write_msg_i[nb_itfs_to_trsmt - 1], &write_byte_i[nb_itfs_to_trsmt - 1],
	  &write_offset[nb_itfs_to_trsmt - 1], NULL, 1, 2);
    }
    // on incremente le compteur de bits
    end_segment_routing++;

    if (itf == END_NODE)
      // si c'est une fin de noeud, on diminue la profondeur dans l'arbre
    {
      depth--;
    }
    else
    {
      // sinon, au contraire, on descend dans l'arbre
      depth++;

      if (depth == 1)
	// si on est à la couche juste en dessous c'est qu'on transmet à un nouveau fils
      {
	nb_itfs_to_trsmt++;
	itfs_to_trsmt[nb_itfs_to_trsmt - 1] = itf;
      }
    }
  }

  // on lit la longueur du message en octet qui se situe juste après le segment routing ++
  // i.e. au tout début du message applicatif, sur le premier octet
  uint8_t length = 0;
  for (int bit_i = 0; bit_i < 8; bit_i++)
  {
    // on lit ce qu'on a stocké
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
    // on recopie pour les interfaces par lesquelles on doit transmettre
    for (int itf_i = 0; itf_i < nb_itfs_to_trsmt; itf_i++)
    {
      msg_to_send[itf_i][write_msg_i[itf_i]][write_byte_i[itf_i]] += value << write_offset[itf_i];
      incr_indexes(&write_msg_i[itf_i], &write_byte_i[itf_i], &write_offset[itf_i], NULL, 1, 2);
    }

    // on determine la longueur du message applicatif
    length += value << (7 - bit_i);
  }

  for (int bit_i = 0; bit_i < length * 8; bit_i++)
    // on réécrit seulement ce qui nous interesse grâce à la longueur du message applicatif
    // (cela évite notamment d'envoyer un sous-message vide à la fin)
  {
    // on lit ce qu'on a stocké
    value = (storage[father_itf][read_msg_i][read_byte_i] & and_op) >> read_offset;
    incr_indexes(&read_msg_i, &read_byte_i, &read_offset, &and_op, 1, 2);
    // on recopie pour les interfaces par lesquelles on doit transmettre
    for (int itf_i = 0; itf_i < nb_itfs_to_trsmt; itf_i++)
    {
      msg_to_send[itf_i][write_msg_i[itf_i]][write_byte_i[itf_i]] += value << write_offset[itf_i];
      incr_indexes(&write_msg_i[itf_i], &write_byte_i[itf_i], &write_offset[itf_i], NULL, 1, 2);
    }
  }

  // le type du message : message to multiple modules
  uint8_t msg_type = MSG_TO_MULT_MODULES;
  // pour chaque interface par laquelle on doit transmettre
  for (int itf_i = 0; itf_i < nb_itfs_to_trsmt; itf_i++)
  {
    // le nombre de sous-messages pour cette transmission
    uint8_t nb_msg = write_msg_i[itf_i] + 1;
    if (write_byte_i[itf_i] == 2 && write_offset[itf_i] == 6 && nb_msg > 1)
      // on a compté un message de trop si l'indice d'écriture est situé au début d'un nouveau
      // sous-message
    {
      nb_msg--;
    }
    for (int msg_i = 0; msg_i <= nb_msg - 1; msg_i++)
      // pour chaque sous-message
    {
      // on ajoute le bon header
      msg_to_send[itf_i][msg_i][0] = (msg_type << 5) + (msg_i << 3) + nb_msg;
      // on ajoute l'id du message
      msg_to_send[itf_i][msg_i][1] = msg_id;
      // on envoie le message
      uint8_t t = Transmit(itfs_to_trsmt[itf_i], msg_to_send[itf_i][msg_i], BUFFSIZE, TIME_OUT);

      if (t == 0)
	// si la transmission a échoué
      {
	// on signale ce problème à la source

	// contenu du message d'erreur
	char* error_text0 = "echec de transmission du message ID";
	char error_msg_id[4];
	sprintf(error_msg_id, "%d", msg_id);
	char * error_text1 = " vers l'interface ITF";
	char error_itf[2];
	sprintf(error_itf, "%d", itfs_to_trsmt[itf_i]);
	char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1)+2);
	strcpy(msg_content, error_text0);
	strcat(msg_content, error_msg_id);
	strcat(msg_content, error_text1);
	strcat(msg_content, error_itf);

	// longueur du message
	const uint8_t length = strlen(msg_content);

	// le message d'erreur
	uint8_t error_msg[length];
	// on remplit avec le contenu du message
	for (int i = 0; i < length; i++)
	{
	  error_msg[i] = (uint8_t) msg_content[i];
	}
	// on envoie le message d'erreur à la source
	Send_Error_Message_to_Source(E_NET_CANT_TRSMT_MSG, error_msg, length);
	break;
      }
    }
  }

  // on renvoie le nombre de bits avant la fin du segment routing ++
  return end_segment_routing;
}


void Handle_Message_to_All(uint8_t *pData)
{
  // les 2 bits après le type de message code le numéro de ce sous-message
  uint8_t sub_msg_number = (pData[0] & 0b00011000) >> 3;
  // les 3 derniers bits du header code le nombre total de sous-messages
  uint8_t sub_msg_total = (pData[0]) & 0b00000111;

  if (msg_to_store[father_itf] == 0 && sub_msg_number == 0)
    // si ce sous-message est le premier
  {
    // on regarde combien de sous-messages on doit stocker
    msg_to_store[father_itf] = sub_msg_total;
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else if (msg_stored[father_itf] == sub_msg_number)
    // si c'est la suite d'un message
  {
    // on stocke le message
    Store_Message(father_itf, pData);
  }
  else
    // si il manque un sous-message
  {
    // on signale ce problème à la source

    // contenu du message d'erreur
    char* error_text0 = "reception du message ID";
    char error_msg_id[4];
    sprintf(error_msg_id, "%d", storage[father_itf][0][1]);
    char * error_text1 = " incomplete";
    char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
    strcpy(msg_content, error_text0);
    strcat(msg_content, error_msg_id);
    strcat(msg_content, error_text1);

    // longueur du message
    const uint8_t length = strlen(msg_content);

    // le message d'erreur
    uint8_t error_msg[length];
    // on remplit avec le contenu du message
    for (int i = 0; i < length; i++)
    {
      error_msg[i] = (uint8_t) msg_content[i];
    }
    // on envoie le message d'erreur à la source
    Send_Error_Message_to_Source(E_NET_UNCOMPLETE_MSG, error_msg, length);

    // on vide le stockage de l'interface du père pour être prêt à recevoir un nouveau message
    Empty_Storage(father_itf);

    if (sub_msg_number == 0)
      // c'est un nouveau message
    {
      // on regarde combien de sous-messages on doit stocker
      msg_to_store[father_itf] = sub_msg_total;
      // on stocke le message
      Store_Message(father_itf, pData);
    }
  }

  if (msg_stored[father_itf] == msg_to_store[father_itf])
    // si on a recu le bon nombre de sous-messages, on peut donc traiter la transmission
  {
    // on transfère tout de suite le message aux fils avant de le lire (c'est un choix modifiable).
    // on imagine par exemple que si il s'agit d'un arrêt d'urgence il vaut mieux
    // que les fils recoivent le message avant que le père s'éteigne.
    Transmit_Message_to_All();

    // on récupère l'id du message situé sur le second octet
    uint8_t msg_id = storage[father_itf][0][1];

    // on réécrit le message dans un unique tableau d'octets
    uint8_t message[(BUFFSIZE - 2) * NB_MAX_SBMSG] = {0};

    // indices de lecture et d'écriture
    int read_msg_i = 0;
    int read_byte_i = 2, write_byte_i = 0;

    // on lit la longueur du message
    uint8_t length = storage[father_itf][read_msg_i][read_byte_i];
    read_byte_i++;

    bool error = false;
    for (int byte_i = 0; byte_i < length; byte_i++)
      // tant que l'on n'a pas lu le message en entier
    {
      if (read_msg_i == msg_stored[father_itf])
	// si on dépasse la fin du message
      {
	// on prévient alors la source de ce problème

	// le contenu du message
	char* error_text0 = "message ID";
	char error_msg_id[4];
	sprintf(error_msg_id, "%d", msg_id);
	char* error_text1 = " illisible";
	char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1));
	strcpy(msg_content, error_text0);
	strcat(msg_content, error_msg_id);
	strcat(msg_content, error_text1);

	// longueur du message
	const uint8_t length = strlen(msg_content);

	// le message d'erreur
	uint8_t error_msg[length];
	// on remplit avec le contenu du message
	for (int i = 0; i < length; i++)
	{
	  error_msg[i] = (uint8_t) msg_content[i];
	}
	// on envoie le message d'erreur à la source
	Send_Error_Message_to_Source(E_NET_UNREADABLE_MSG, error_msg, length);
	error = true;
	break;
      }
      // on réécrit dans le tableau du message
      message[write_byte_i] = storage[father_itf][read_msg_i][read_byte_i];

      // gestion des indices d'écriture
      write_byte_i++;
      // gestion des indices de lecture
      if (read_byte_i == BUFFSIZE - 1)
      {
	read_byte_i = 2;
	read_msg_i++;
      }
      else
      {
	read_byte_i++;
      }
    }

    if (!error)
    {
      // on peut maintenant lire le message
      Read_Message(msg_id, message, length);
    }

    // puis on vide le stockage de l'interface du père
    Empty_Storage(father_itf);
  }
}


void Transmit_Message_to_All()
{
  // l'id du message
  uint8_t msg_id = storage[father_itf][0][1];
  // le nombre de sous-messages pour cette transmission qu'on récupère directement dans le header.
  uint8_t nb_msg = (storage[father_itf][0][0]) & 0b00000111;

  // pour chaque fils
  for(int son_i = 0; son_i < son_nb; son_i++)
  {
    // on récupère l'interface du fils
    uint8_t son_itf = son_itfs[son_i];

    for (int msg_i = 0; msg_i < nb_msg; msg_i++)
    {
      // on transfère le message au fils
      uint8_t t = Transmit(son_itf, storage[father_itf][msg_i], BUFFSIZE, TIME_OUT);

      if (t == 0)
	// si la transmission a échoué
      {
	// on signale ce problème à la source

	// contenu du message d'erreur
	char* error_text0 = "echec de transmission du message ID";
	char error_msg_id[4];
	sprintf(error_msg_id, "%d", msg_id);
	char * error_text1 = " vers l'interface ITF";
	char error_itf[2];
	sprintf(error_itf, "%d", son_itf);
	char* msg_content = malloc(strlen(error_text0)+4+strlen(error_text1)+2);
	strcpy(msg_content, error_text0);
	strcat(msg_content, error_msg_id);
	strcat(msg_content, error_text1);
	strcat(msg_content, error_itf);

	// longueur du message
	const uint8_t length = strlen(msg_content);

	// le message d'erreur
	uint8_t error_msg[length];
	// puis on remplit avec le contenu du message
	for (int i = 0; i < length; i++)
	{
	  error_msg[i] = (uint8_t) msg_content[i];
	}
	// on envoie le message d'erreur à la source
	Send_Error_Message_to_Source(E_NET_CANT_TRSMT_MSG, error_msg, length);
	break;
      }
    }
  }
}


void Store_Message(uint8_t itf, uint8_t *pData)
{
  // on copie les données sur le tableau de stockage correspondant à l'interface itf
  for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
  {
    storage[itf][msg_stored[itf]][byte_i] = pData[byte_i];
  }

  // on actualise cette variable
  msg_stored[itf]++;
}


void Empty_Storage(uint8_t itf)
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

// ==================================== COUCHE APPLICATIVE =========================================

void Read_Message(uint8_t msg_id, uint8_t *pData, uint8_t length)
{
  // TODO : à compléter
}

void Send_Error_Message_to_Source(uint8_t error_code, uint8_t *error_msg, uint8_t length)
{
  // TODO : à compléter
}

// ===================================== FONCTIONS UTILES ==========================================

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

