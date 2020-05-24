#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UNKNOWN_ID 0
#define NB_ITF 3
#define UNKNOWN_ITF 4
#define BUFFSIZE 32
#define NB_MAX_SBMSG 4
#define INIT 0
#define INIT_R 1
#define MSG_TO_MODULE 2
#define MSG_TO_SOURCE 3
#define TIME_OUT 500
#define END_NODE 3
#define END_SEGMENT_ROUTING 3

/* --- INFORMATIONS IMPORTANTES ---
 * Le protocole de communication est codé de façon à pouvoir transmettre des messages répartis en
 * un maximum de NB_MAX_SB_MSG sous-messages, tous de taille BUFFSIZE octets. De plus, lors de la
 * transmission d'un message à un module, le premier octet du premier sous-message (le deuxième en
 * comptant l'octet réservé à l'header) doit toujours indiquer la taille du message applicaatif en octet
 * sans prendre en compte ce même octet.
 * Ci-dessous sont presentés les différentes fonctions utiles au protocole de communication. */

/* --- Réception de message --- 
 * Renvoie 0 en cas d'erreur et 1 en cas de succès : la fonction termine avec succès quand elle recoit
 * au moins length octet sur l'interface itf. Elle recopie alors les length octet recus dans
 * la memoire à partir du pointeur pData. Cette méthode n'est pas bloquante (ie le programme continue de
 * fonctionner après l'appel de cette fonction sans attendre que la fonction ait terminée). 
 * Quand la fonction termine sans erreur, elle appelle automatiquement la fonction RxCallback. */
uint8_t Receive_IT(uint8_t itf, uint8_t *pData, uint8_t length);

/* --- RxCallBack ---
 * Cette fonction est appelée dès que la fonction Receive_IT termine sans erreur 
 * (ie dès qu'un message a été recu sur l'interface itf). L'argument id correspond à l'identifiant 
 * de l'interface sur laquelle le message a été reçu.  */
uint8_t RxCallback(uint8_t itf);

/* --- Envoi de message --- 
 * Renvoie 0 en cas d'erreur et 1 en cas de succès : En cas de succès, transmet length octet contenus
 * dans la memoire à partir du pointeur pData sur l'interface itf. Si le message n'est pas reçu par 
 * le recepteur au bout de Timeout ms, l'envoi s'arrête et la fonction renvoie 0. */
uint8_t Transmit(uint8_t itf, uint8_t *pData, uint8_t length, uint16_t Timeout);

/* --- Stockage de sous-messages ---
 * Cette fonction permet de stocker un message provenant de l'interface itf et contenu dans la
 * mémoire à partir du pointeur pData. La taille d'un message est toujours de taille BUFFSIZE
 * octes. */
void Store_Message(uint8_t itf, uint8_t *pData);

/* --- Vider le stockage ---
 * Cette fonction permet de vider le stockage de messages provenant de l'interface itf. */
void Empty_Storage(uint8_t itf);

/* --- Gestion d'un message ---
 * Cette fonction est appelée lors de la réception d'un message depuis l'interface itf et
 * contenu dans la mémoire à partir du pointeur pData. Elle a pour but de rediriger les donnees
 * réceptionnées vers la fonction adaptee au type du message recu. */
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
 * que le module n'a pas de fils. Elle a pour but d'écrire l'init_r et de l'envoyer au père du module. */
void Send_init_r();

/* --- Gestion d'un message pour un module ---
 * Cette fonction est appelée lors de la réception d'un message destiné à un module. Elle a pour
 * but de stocker le message et lorsqu'une transmission a ete reçue complètement, d'appeler soit la
 * fonction Read_Message si le message est destiné a ce module, soit la fonction
 * Transfer_Message_to_Module sinon. */
void Handle_Message_to_Module(uint8_t *pData);

/* --- Transfert d'un message à un autre module ---
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné a un autre
 * module (un module fils). Elle a pour but de réécrire le message à transmettre et l'envoyer au bon
 * fils grâce au principe du segment routing. */
void Transfer_Message_to_Module();

/* --- Lecture d'un message ---
 * Cette fonction est appelée lorsque l'on a reçu entierement un message destiné a ce module.
 * Elle a pour but de lire le message et d'exécuter certaines tâches en fonction de son contenu. */
void Read_Message(uint8_t *pData);

/* --- Gestion d'un message pour la source ---
 * Cette fonction est appelée lors de la réception d'un message destiné a la source. Elle a pour
 * but transférer le message au père du module. */
void Handle_Message_to_Source(uint8_t *pData);

/* --- Envoi d'un message à la source ---
 * Cette fonction peut être appelée lorsque l'on souhaite envoyer un message à la source.
 * IMPORTANT : le premier octet de pData donne la longueur du message qui le suit en octet. */
void Send_Message_to_Source(uint8_t *pData);

/* --- Comparaison de 2 tableaux ---
 * Renvoie 1 si les 2 tableaux sont identiques, 0 sinon. */
int compareArrays(uint8_t *a, uint8_t *b, int size);

/* --- Incrémentation des indices ---
 * Cette fonction est utile pour incrémenter facilement des indices lors de la lecture ou l'écriture
 * de messages. */
void incr_indexes (int *msg_i, int *byte_i, int *offset, uint8_t *and_op, int incr);
