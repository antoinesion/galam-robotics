#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UNKNOWN_ID 0 // code pour un identifiant inconnu
#define NB_ITF 3 // nombre d'interfaces sur un module
#define UNKNOWN_ITF 4 // code pour une interface inconnue
#define BUFFSIZE 32 // taille en octet des transmissions
#define NB_MAX_SBMSG 4 // nombre maximal de sous-messages
#define UNKNOWN_INIT_ID 0 // code pour un id d'init inconnu
#define INIT 0 // code pour un message de type init
#define INIT_R 1 // code pour un message de type init_r
#define IDENTIFICATION 2 // code pour un message d'identification
#define MSG_TO_MODULE 3 // code pour un message destiné à un module
#define MSG_TO_SOURCE 4 // code pour un message destiné à la source
#define MSG_TO_MULT_MODULES 5 // code pour un message destiné à plusieurs modules
#define MSG_TO_ALL 6 // code pour un message destiné à tous les modules
#define TIME_OUT 500 // temps maximal d'attente pour la transmission
#define END_NODE 3 // code pour signaler une fin de noeud dans l'init_r / segment routing ++
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
 * Le protocole de communication est codé de façon à pouvoir transmettre des messages répartis en
 * un maximum de NB_MAX_SB_MSG sous-messages, tous de taille BUFFSIZE octets.
 * Ci-dessous sont presentés les différentes fonctions utiles au protocole de communication. */

// ======================================= COUCHE RÉSEAU ============================================

/* --- Réception de message --- 
 * Renvoie 0 en cas d'erreur et 1 en cas de succès : la fonction termine avec succès quand elle recoit
 * au moins length octet sur l'interface itf. Elle recopie alors les length octet recus dans
 * la memoire à partir du pointeur pData. Cette méthode n'est pas bloquante (ie le programme continue de
 * fonctionner après l'appel de cette fonction sans attendre que la fonction ait terminée). 
 * Quand la fonction termine sans erreur, elle appelle automatiquement la fonction RxCallback. */
int Receive_IT(uint8_t itf, uint8_t *pData, uint8_t length);

/* --- RxCallBack ---
 * Cette fonction est appelée dès que la fonction Receive_IT termine sans erreur 
 * (ie dès qu'un message a été recu sur l'interface itf). L'argument id correspond à l'identifiant 
 * de l'interface sur laquelle le message a été reçu. */
int RxCallback(uint8_t itf);

/* --- Transmission d'un message --- 
 * Renvoie 0 en cas d'erreur et 1 en cas de succès : En cas de succès, transmet length octet contenus
 * dans la memoire à partir du pointeur pData sur l'interface itf. Si le message n'est pas reçu par 
 * le recepteur au bout de Timeout ms, l'envoi s'arrête et la fonction renvoie 0. */
int Transmit(uint8_t itf, uint8_t *pData, uint8_t length, uint16_t Timeout);

/* --- Gestion d'un message ---
 * Cette fonction est appelée lors de la réception d'un message depuis l'interface itf et
 * contenu dans la mémoire à partir du pointeur pData. Elle a pour but de rediriger les données
 * réceptionnées vers la fonction adaptee au type du message reçu. */
void Handle_Message(uint8_t itf, uint8_t *pData);

/* --- Gestion d'un message init ---
 * Cette fonction est appelée lors de la récepetion d'un message de type init recu depuis
 * l'interface itf. Elle a pour but de transférer l'init au fils du module et d'initialiser
 * certaines variables. */
void Handle_Message_init(uint8_t itf, uint8_t *pData);

/* --- Gestion d'un message init_r ---
 * Cette fonction est appelée lors de la réception d'un message de type init_r par l'un des fils
 * du module (depuis l'interface itf). Elle a pour but de stocker ce message et d'éventuellement
 * appeler la fonction Send_init_r. */
void Handle_Message_init_r(uint8_t itf, uint8_t *pData);

/* --- Envoi du message init_r ---
 * Cette fonction est appelée lorsque tous les messages d'init_r des fils ont bien été reçus ou
 * que le module n'a pas de fils. Elle a pour but d'écrire l'init_r et de l'envoyer au père du module.
 * Renvoie 0 en cas d'erreur et 1 en cas de succès. */
int Send_init_r();

/* --- Gestion d'un message d'identification ---
 * Cette fonction est appelée lors de la réception d'un message d'identification d'un module. Elle a
 * pour but de transmettre le message avec la fonction Transfer_Message_Identification si il l'identifiant
 * n'est pas destiné à ce module ou alors changer l'identifiant de ce module sinon. */
void Handle_Message_Identification(uint8_t *pData);

/* --- Transmission d'un message d'identification ---
 * Cette fonction est appelée lorsque l'on a reçu un message d'identification destiné à un autre
 * module. Elle a pout but de réécrire le message à transmettre et l'envoyer au bon fils grâce au
 * principe du segment routing. */
void Transmit_Message_Identification();

/* --- Gestion d'un message pour un module ---
 * Cette fonction est appelée lors de la réception d'un message destiné à un module. Elle a pour
 * but de stocker le message et lorsqu'une transmission a ete reçue complètement, d'appeler soit la
 * fonction Read_Message si le message est destiné a ce module, soit la fonction
 * Transfer_Message_to_Module sinon. */
void Handle_Message_to_Module(uint8_t *pData);

/* --- Transmission d'un message à un autre module ---
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné à un autre
 * module. Elle a pour but de réécrire le message à transmettre et l'envoyer au bon fils grâce au
 * principe du segment routing. */
void Transmit_Message_to_Module();

/* --- Gestion d'un message pour la source ---
 * Cette fonction est appelée lors de la réception d'un message destiné à la source. Elle a pour
 * but transférer le message au père du module. */
void Handle_Message_to_Source(uint8_t *pData);

/* --- Envoi d'un message à la source ---
 * Cette fonction peut être appelée lorsque l'on souhaite envoyer un message à la source. */
void Send_Message_to_Source(uint8_t *pData, uint8_t length);

/* --- Gestion d'un message pour plusieurs modules ---
 * Cette fonction est appelee lors de la réception d'un message destiné à plusieurs modules. Elle a
 * pour but d'appeler la fonction Read_Message si le message est destiné à ce module et de
 * transférer le message après de la réception complète de ce dernier. */
void Handle_Message_to_Multiple_Modules(uint8_t *pData);

/* --- Transmission d'un message pour plusieurs modules --- 
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné à plusieurs
 * modules. Elle a pour but de réécrire le message à transmettre et l'envoyer aux bons fils grâce au
 * principe du segment routing. Renvoie la position de la fin du segment routing ou 0 en cas
 * d'erreur. */
int Transmit_Message_to_Multiple_Modules();

/* --- Gestion d'un message pour tous les modules ---
 * Cette fonction est appelée lors de la réception d'un message destiné à tous les modules. Elle a
 * pour but d'appeler la fonction Read_Message et la fonction Transfer_Message_to_all après la
 * réception complète du message. */
void Handle_Message_to_All(uint8_t *pData);

/* --- Transmission d'un message pour tous les modules ---
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné à tous les modules.
 * Elle a pour but de transferer tel quel le message aux fils du module. */
void Transmit_Message_to_All();

/* --- Stockage de sous-messages ---
 * Cette fonction permet de stocker un message provenant de l'interface itf et contenu dans la
 * mémoire à partir du pointeur pData. La taille d'un message est toujours de taille BUFFSIZE
 * octets. */
void Store_Message(uint8_t itf, uint8_t *pData);

/* --- Vider le stockage ---
 * Cette fonction permet de vider le stockage de messages provenant de l'interface itf. */
void Empty_Storage(uint8_t itf);

// ==================================== COUCHE APPLICATIVE =========================================

/* --- Lecture d'un message ---
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné à ce module.
 * Elle a pour but de lire le message et d'exécuter certaines tâches en fonction de son contenu.
 * Elle prend en argument l'identifiant du message, le message et la longueur du message en octet. */
void Read_Message(uint8_t msg_id, uint8_t *pData, uint8_t length);

/* --- Envoi de message d'erreur à la source ---
 * Cette fonction peut être appelée lorsqu'une erreur survient dans l'exécution d'une tâche. Elle a
 * pour but de transmettre l'erreur à la source. Elle prend en argument un code d'erreur, un
 * message d'erreur et la longueur de ce message en octet. */
void Send_Error_Message_to_Source(uint8_t error_code, uint8_t *error_msg, uint8_t length);

// ===================================== FONCTIONS UTILES ==========================================

/* --- Comparaison de 2 tableaux ---
 * Renvoie 1 si les 2 tableaux sont identiques, 0 sinon. */
int compareArrays(uint8_t *a, uint8_t *b, int size);

/* --- Incrémentation des indices ---
 * Cette fonction est utile pour incrémenter facilement des indices lors de la lecture ou l'écriture
 * de messages. */
void incr_indexes (int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr, int byte_start);
