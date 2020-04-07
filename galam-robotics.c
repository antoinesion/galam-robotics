#include "galam-robotics.h"

/*
Variables
Par convention, l'interface 0 est l'interface amont, et les interfaces avales sont les interfaces 1 et 2. 
Les messages en provenance de l'interface 0 seront donc recus dans le tableau rx0 et 
les messages à envoyer vers l'interface 0 seront stockés dans le tableau tx0. 
*/

#define BUFFSIZE 8 //Taille des messages en octet, A MODIFIER SI BESOINS
uint8_t tx0[BUFFSIZE] ;
uint8_t rx0[BUFFSIZE] ;
uint8_t tx1[BUFFSIZE] ;
uint8_t rx1[BUFFSIZE] ;
uint8_t tx2[BUFFSIZE] ;
uint8_t rx2[BUFFSIZE] ;
/*
TODO: variables pour stocker des informations comme l'id du pere, les ids des fils,
l'init-r d'un des fils en attendant l'autre, etc.
*/

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

void main(){
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
}

void Handle_Message(uint8_t *pData, uint8_t id)
{
    //TODO: rediriger vers Handle_Message_init ou Handle_Message_send en fonction de l'entete
}

void Handle_Message_init(uint8_t *pData, uint8_t id)
{
    //TODO
}

void Handle_Message_send(uint8_t *pData)
{
    //TODO
}
