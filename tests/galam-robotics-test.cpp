#include "galam-robotics-module.hpp"
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>

const int NB_MODULES = 10;
std::vector<uint8_t> paths[NB_MODULES];
uint8_t entry_id = UNKNOWN_ID;

/* fonction pour generer un arbre de maniere aleatoire */
void Generer_Arbre(std::vector<Module*> modules)
{
  if (modules.size() > 1)
  {
    int nb_son = (1 - (rand() % 3) % 2) + 1;
    if (nb_son == 1 || modules.size() < 3)
    {
      Module* father = modules[0];
      std::vector<Module*> sons (modules.begin() + 1, modules.end());
      Generer_Arbre(sons);
      
      uint8_t first_id_father_side = father->get_random_id();
      uint8_t first_id_son_side = sons[0]->get_random_id();
      father->set_first_son(sons[0], first_id_father_side, first_id_son_side);
      sons[0]->set_father(father, first_id_son_side, first_id_father_side);
    }
    else
    {
      Module* father = modules[0];
      int first_sons_nb = rand() % modules.size();
      std::vector<Module*> first_sons (modules.begin() + 1, modules.begin() + 1 + first_sons_nb);
      std::vector<Module*> second_sons (modules.begin() + 1 + first_sons_nb, modules.end());
      Generer_Arbre(first_sons);
      Generer_Arbre(second_sons);

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
    }
  }
}

/* fonction pour envoyer un message a un module */
void Send_Message(std::string text, int module_id, Module* source)
{
  uint8_t length = text.length();
  uint8_t message[NB_MAX_MSG][BUFFSIZE] = {0};
  int write_msg_i = 0;
  int write_byte_i = 1;
  int write_offset = 6;
  // segment routing
  std::cout << "segement routing: ";
  for (uint8_t id : paths[module_id])
  {
    std::cout << unsigned(id);
    message[write_msg_i][write_byte_i] += id << write_offset;
    incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
  }
  std::cout << unsigned(END_HEADER) << std::endl;
  message[write_msg_i][write_byte_i] += END_HEADER << write_offset;
  incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);

  // message length
  uint8_t and_op = 0b11000000;
  for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
  {
    message[write_msg_i][write_byte_i] += ((length&and_op) >> read_offset) << write_offset;
    incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
    and_op = and_op >> 2;
  }
  // message
  for (int i = 0 ; i < length ; i++)
  {
    uint8_t and_op = 0b11000000;
    for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
    {
      message[write_msg_i][write_byte_i] += ((((uint8_t) text[i])&and_op) >> read_offset) << write_offset;
      incr_iterators(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
      and_op = and_op >> 2;
    }
  }

  // header & send
  uint8_t nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
  {
    message[msg_i][0] = (MSG_TO_SON << 6) + nb_msg;
/*     for (int byte_i = 0; byte_i < BUFFSIZE; byte_i++)
 *     {
 *       std::bitset<8> byte (message[msg_i][byte_i]);
 *       std::cout << byte << " ";
 *       if (byte_i % 4 == 3)
 *       {
 *         std::cout << std::endl;
 *       }
 * 
 *     } */
    source->Send_Message(entry_id, message[msg_i]);
  }

}

/* fonction pour lire l'init_r */
void Read_init_r(std::string init_r)
{
  std::vector<uint8_t> stack;
  int i = 0;
  int module_id = 1;
  int8_t value = ((int8_t) init_r[i]) - 48;
  while (stack.size() > 0 || value != END_NODE)
  {
    if (value == END_NODE)
    {
      stack.pop_back();
    }
    else
    {
      stack.push_back(value);
      paths[module_id] = stack;
      module_id++;
    }
    i++;
    value = ((int8_t) init_r[i]) - 48;
  }
}

int main()
{
  // initialisation de la fonction srand (pour les nombres aleatoires)
  srand (time(NULL));
  // on vide le fichier init_r.txt
  std::ofstream file;
  file.open("init_r.txt");
  file << "";
  file.close();

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
  while (loop)
  {
    // affichage de l'arbre
    std::cout << std::endl;
    modules[0].print();
    std::cout << std::endl;
    std::string aswr;
    std::getline(std::cin, aswr);
    
    if (aswr == "stop")
    {
      loop = false;
    }
    else if (aswr == "send init")
    {
      entry_id = modules[0].Send_init();
      std::cout << "init sent!" << std::endl;
    }
    else if (aswr == "read init_r")
    {
      modules[0].naming();
      std::ifstream file;
      file.open("init_r.txt");
      std::string init_r;
      file >> init_r;
      if (init_r == "")
      {
	if (entry_id == UNKNOWN_ID)
	{
	  std::cout << "init hasn't been sent" << std::endl;
	}
	else
	{
	  std::cout << "init_r hasn't been received yet" << std::endl;
	}
      }
      else
      {
	Read_init_r(init_r);
	std::cout << "init_r sucessfully read, you can now send messages!" << std::endl;
      }
      file.close();
    }
    else if (aswr.find("send") == 0)
    {
      aswr = aswr.substr(aswr.find(" ")+1);
      int module_id = std::stoi(aswr.substr(0, aswr.find(" ")));
      std::string text  = aswr.substr(aswr.find(" ")+1);
      Send_Message(text, module_id, modules);
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
