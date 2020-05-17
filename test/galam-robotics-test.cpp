#include "galam-robotics-module.hpp"
#include <time.h>
#include <vector>

/* fonction pour generer un arbre de maniere aleatoire */
int Generer_Arbre(std::vector<Module*> modules, int module_id = 0)
{
  if (modules.size() > 1)
  {
    int nb_son = (1 - (rand() % 3) % 2) + 1;
    if (nb_son == 1 || modules.size() < 3)
    {
      Module* father = modules[0];
      father->module_id = module_id;
      std::vector<Module*> sons (modules.begin() + 1, modules.end());
      module_id = Generer_Arbre(sons, ++module_id);
      
      uint8_t first_id_father_side = father->get_random_id();
      uint8_t first_id_son_side = sons[0]->get_random_id();
      father->set_first_son(sons[0], first_id_father_side, first_id_son_side);
      sons[0]->set_father(father, first_id_son_side, first_id_father_side);
      
      return module_id;
    }
    else
    {
      Module* father = modules[0];
      father->module_id = module_id;
      int first_sons_nb = rand() % modules.size();
      std::vector<Module*> first_sons (modules.begin() + 1, modules.begin() + 1 + first_sons_nb);
      std::vector<Module*> second_sons (modules.begin() + 1 + first_sons_nb, modules.end());
      module_id = Generer_Arbre(first_sons, ++module_id);
      module_id = Generer_Arbre(second_sons, module_id);

      if (first_sons.size() > 0)
      {
	uint8_t first_id_father_side = father->get_random_id();
	uint8_t first_id_son_side = first_sons[0]->get_random_id();
	father->set_first_son(first_sons[0], first_id_father_side, first_id_son_side);
	first_sons[0]->set_father(father, first_id_son_side, first_id_father_side);
      }

      if (second_sons.size() > 0)
      {
	uint8_t second_id_father_side = father->get_random_id();
	uint8_t second_id_son_side = second_sons[0]->get_random_id();
	father->set_second_son(second_sons[0], second_id_father_side, second_id_son_side);
	second_sons[0]->set_father(father, second_id_son_side, second_id_father_side);
      }
      
      return module_id;
    }
  }
  else if (modules.size() == 1)
  {
    modules[0]->module_id = module_id;
    return ++module_id;
  }
  else
  {
    return module_id;
  }
}

const int NB_MODULES = 10;

int main()
{
  // initialisation de la fonction srand (pour les nombres aleatoires)
  srand (time(NULL));

  // creation des modules
  Module modules[NB_MODULES];
  std::vector<Module*> modules_ptr;
  for (int i = 0 ; i < NB_MODULES ; i++)
  {
    modules_ptr.push_back(&modules[i]);
  }
  // generation de l'arbre
  Generer_Arbre(modules_ptr);
  
  bool loop = true;
  uint8_t open_id;
  while (loop)
  {
    // affichage de l'arbre
    modules[0].print();
    std::string aswr;
    std::cin >> aswr;
    if (aswr == "init")
    {
      open_id = modules[0].Send_init();
    }
    
    if (aswr == "stop")
    {
      loop = false;
    }
    else
    {
      for (int i = 0; i < NB_MODULES ; i++)
      {
	modules[i].Handle_All_Message();
      }
    }
  }
}
