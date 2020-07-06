#include <stdint.h>
#include <vector>

#define BUFFSIZE 32 // taille en octet des transmissions
#define TIME_OUT 500 // temps maximal d'attente pour la transmission
#define END_NODE 3 // code pour signaler une fin de noeud dans l'init_r / segment routing ++
#define NB_MAX_SBMSG 4 // nombre maximal de sous-messages
#define INIT 0 // code pour un message de type init
#define IDENTIFICATION 2 // code pour un message d'identification
#define MSG_TO_MODULE 3 // code pour un message destiné à un module
#define MSG_TO_MULT_MODULES 5 // code pour un message destiné à plusieurs modules
#define MSG_TO_ALL 6 // code pour un message destiné à tous les modules
#define END_SEGMENT_ROUTING 3 // code pour signaler la fin du segment routing dans un message

// codes d'erreur de la couche réseau
#define E_NET_UNKNOWN_MSG_TYPE 101 // type de message inconnu
#define E_NET_UNCOMPLETE_MSG 102 // message incomplet
#define E_NET_CANT_TRSMT_ID 111 // impossible de transmettre un message d'identification
#define E_NET_CANT_TRSMT_MSG 112 // impossible de transmettre un message classique
#define E_NET_UNREADABLE_INIT_R 121 // init_r illisible
#define E_NET_UNREADABLE_SEG_RT 122 // segment routing illisible
#define E_NET_UNREADABLE_MSG 123 // message illisible

/* --- INFORMATIONS ---
 * Ce fichier propose quelques fonctions utiles pour la source. Pour que ces fonctions soient
 * opérationnelles, la fonction Transmit ci-dessous doit cependant être codée. */

/* --- Transmission d'un message au premier module ---
 * Renvoie 0 en cas d'erreur et 1 en cas de succès : En cas de succès, transmet length octet contenus
 * dans la memoire à partir du pointeur pData vers le premier module. Si le message n'est pas reçu par 
 * le recepteur au bout de Timeout ms, l'envoi s'arrête et la fonction renvoie 0. */
int Transmit(uint8_t *pData, uint8_t length, uint16_t Timeout);

/* --- Envoi d'un init ---
 * Cette fonction envoi le message d'initialisation. Elle prend en argument un identifiant d'init
 * qui doit être différent de 0 et d'un potentiel identifiant d'init précédent ! Renvoie 0 en cas
 * d'erreur et 1 en cas de succès. */
int Send_init(uint8_t init_id);

/* --- Lecture de l'init_r ---
 * Cette fonction peut être appelée lorsque la source a reçu l'init_r. Elle permet de remplir le
 * tableau segment_routings qu'elle prend en argument grâce l'init_r mis sous la forme d'un tableau
 * d'octets (sans les header). */
void Read_init_r(uint8_t *init_r, std::vector<uint8_t> *segment_routings);

/* --- Envoi d'un message d'identification ---
 * Cette fonction peut être appelée après la lecture de l'init_r pour identifier les modules,
 * les uns à la suite des autres. Attention à ne pas identifier tous les modules d'un coup car cela
 * pourrait surcharger le réseau. La fonction prend en argument un identifiant de module ainsi que
 * le segment routing pour y accéder. Renvoie 0 en cas d'erreur et 1 en cas de succès. */
int Send_Message_Identification(uint16_t module_id, std::vector<uint8_t> segment_routing);

/* --- Envoi d'un message à un module ---
 * Cette fonction peut être appelée lorsque l'on souhaite envoyer un message à un module. Elle prend
 * en argument le message sous forme d'un tableau d'octets, la longueur du message en octets,
 * l'identifiant du message et le segment routing pour accèder au module. Renvoie 0 en cas d'erreur et
 * 1 en cas de succès. */
int Send_Message_to_Module(uint8_t *msg, uint8_t length, uint8_t msg_id, std::vector<uint8_t> segment_routing);

/* --- Envoi d'un message à plusieurs modules ---
 * Cette fonction peut être appelée lorsque l'on souhaite envoyer un même message à plusieurs
 * modules. Elle prend en argument le message sous forme d'un tableau d'octets, la longueur du
 * message en octets, l'identifiant du message et un vecteur de segment routings pour accéder aux
 * modules auxquels le message est destiné. Renvoie 0 en cas d'erreur et 1 en cas de succès. */
int Send_Message_to_Multiple_Modules(uint8_t *msg, uint8_t length, uint8_t msg_id, std::vector<std::vector<uint8_t>> segment_routings);

/* --- Envoi d'un message à tous les modules ---
 * Cette fonction peut être appelée lorsque l'on souhaire envoyer un même message à tous les
 * modules. Elle prend en argument le message sous forme d'un tableau d'octets, la longueur du
 * message en octets et l'identifiant du message. Renvoie 0 en cas d'erreur et 1 en cas de succès. */
int Send_Message_to_All(uint8_t *msg, uint8_t length, uint8_t msg_id);

/* --- Incrémentation des indices ---
 * Cette fonction est utile pour incrémenter facilement des indices lors de la lecture ou l'écriture
 * de messages. */
void incr_indexes (int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr, int byte_start);

/* --- Tri de segment routings ---
 * Cette fonction est utile dans Send_Message_to_Multiple_Modules pour trier le vecteur de segment
 * routings. */
bool sort_segment_routings (std::vector<uint8_t> seg_rt1, std::vector<uint8_t> seg_rt2);
