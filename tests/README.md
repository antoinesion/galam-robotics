# Simulation virtuelle de modules

### Configuration & Compilation

1. Choissisez combien de modules, vous voulez simuler en modifiant la constante `NB_MODULES` dans
   `galam-robotics-tests.cpp`.
2. Compiler les fichiers en exécutant la commande `make` dans votre terminal.

### Tests

*Attention : il est recommandé de mettre la fenêtre de votre terminal en plein écran pour une
meilleure lisibilité.*

Exécuter la commande `./test` dans votre terminal pour lance une simulation aléatoire d'un nombre de
modules que vous avez définis lors de la configuration.

#### Présentation

La simulation virtuelles se présente sous forme d'un arbre écrit horizontalement ; chaque module est
une ligne et ses éventuels modules fils sont situés en dessous, décalé par une tabulation.

Un module se présente sous la forme suivante :

```
[itf_father_side]<->[itf_module_side] MOD[id] [state] [nb_msg]M
```

* `[itf_father_side]` est un nombre (0, 1 ou 2) connu après initialisation du module et qui correspond à
  l'interface du module père par lequel il est connecté.
* `[itf_module_side]` est un nombre (0, 1 ou 2) connu après initialisation du module et qui
  correspond à l'interface du module par lequel le module père est connecté.
* `[id]` est un nombre connu après initialisation et qui représente l'identifiant du module.
* `[state]` est l'état actuel du module. Il peut être :
  * `NI` pour non initialisé
  * `I` pour initialisé
  * `IR` pour initialisé et ayant renvoyé l'init_r.
* `[nb_msg]` est le nombre de message en attente de lecture par le module.

#### Phase d'initialisation & identification

Afin d'initialiser le réseau de modules, vous devez suivre les instructions suivantes, **dans
l'ordre** :
* Taper `send init` pour envoyer le message d'initialisation
* Appuyez ensuite, à plusieurs reprises, sur la touche `ENTER` de votre clavier pour que l'init se
  propage dans l'arbre que l'init_r revienne jusqu'à la source.
* Une fois que l'init a été réçu par la source, utilisr la commande `read init_r`
* Le processus d'identification se lance automatiquement. Appuyez, à plusieurs reprises, sur la
  touche `ENTER` de votre clavier jusqu'a ce que aucun module n'est de messages en attente.

Vous pouvez dès maintenant envoyer des messages aux modules en vous référant à la section suivante.

#### Envoi de messages aux modules

Pour envoyer des messages aux modules, 2 options s'offrent à vous : 
* Pour envoyer un message à un module ou plusieurs modules, utilisez la commande `send [module_ids] [msg]` en
  remplaçant `[module_ids]` par le ou les identifiant(s) des modules auquels vous souhaitez envoyer le message 
  **(séparé par des esapces)** et `[msg]` par le message en question **(sans accents)**.
* Pour envoyer un même message à tous les modules, utilisez la commande `send all [msg]` en
  remplaçant `[msg]` par votre message.
