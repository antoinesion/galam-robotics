#include <stdint.h>
#define BUFFSIZE 32
#define NB_MAX_MSG 4
#define TIME_OUT 500
#define UNKNOWN_ID 4
#define END_NODE 3
/*
Ce fichier header décrit certaines fonctions utilisées dans le code. 
N'hesitez pas à en rajouter !
*/

/*
Fonction de réception de message. 
Renvoie 0 en cas d'erreur et 1 en cas de succès
La fonction termine avec succès quand elle recoit au moins length octet sur l'interface d'identifiant id. 
Elle recopie alors les length octet recus dans la memoire à partir du pointeur pData. 
Cette méthode n'est pas bloquante (ie le programme continue de fonctionner après
l'appel de cette fonction sans attendre que la fonction ait terminée). 
Quand la fonction termine sans erreur, elle appelle automatiquement la fonction RxCallback.
*/
uint8_t Receive_IT(uint8_t id, uint8_t *pData, uint16_t length ) ;

/*
Cette fonction est appelée dès que la fonction Receive_IT termine sans erreur 
(ie dès qu'un message a été recu sur l'interface id). L'argument id correspond à l'identifiant 
de l'interface sur laquelle le message a été recu. 
*/
uint8_t RxCallback(uint8_t id) ;

/*
Fonction d'envoi de message. 
Renvoie 0 en cas d'erreur et 1 en cas de succès
En cas de succès, transmet length octet contenus dans la memoire à partir du pointeur pData
sur l'interface d'identifiant id. Si le message n'est pas recu par le recepteur au bout de Timeout ms,
l'envoi s'arrête et la fonction renvoie 0.
*/
uint8_t Transmit(uint8_t id, uint8_t *pData, uint16_t length, uint32_t Timeout) ;

/*
Fonctions de gestion de message. Le protocole doit être implémenté dedans. 
Il est possible de changer le prototype de la fonction si cela semble pertinent,
il faudra alors modifier les autres fonctions
*/
void Store_Message(uint8_t *pData, int id, int byte_i, int bit_offset);

void emptyStorage(int id);

void Handle_Message(uint8_t *pData, uint8_t id);

void Handle_Message_init(uint8_t *pData, uint8_t id);

void Handle_Message_init_r(uint8_t *pData, uint8_t id);

void Send_init_r();

void Handle_Message_to_son(uint8_t *pData);

void Handle_Message_to_father(uint8_t *pData);

/*
Fonctions utilitaires
*/
int compareArrays(uint8_t *a, uint8_t *b, int size);

