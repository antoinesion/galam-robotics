#include "galam-robotics-module.hpp"
#include <fstream>
#include <iomanip>
#include <string>
#include <time.h>
#include <vector>

const int NB_MODULES = 20;
std::vector<uint8_t> segment_routing[NB_MODULES];
uint8_t entry_itf = UNKNOWN_ITF;
bool id_process_started = false;
uint8_t id_process_id = 0;

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

      uint8_t first_itf_father_side = father->get_random_itf();
      uint8_t first_itf_son_side = sons[0]->get_random_itf();
      father->set_first_son(sons[0], first_itf_father_side, first_itf_son_side);
      sons[0]->set_father(father, first_itf_son_side, first_itf_father_side);
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
	uint8_t first_itf_father_side = father->get_random_itf();
	uint8_t first_itf_son_side = first_sons[0]->get_random_itf();
	father->set_first_son(first_sons[0], first_itf_father_side, first_itf_son_side);
	first_sons[0]->set_father(father, first_itf_son_side, first_itf_father_side);
      }

      if (second_sons.size() > 0)
      {
	uint8_t second_itf_father_side = father->get_random_itf();
	uint8_t second_itf_son_side = second_sons[0]->get_random_itf();
	father->set_second_son(second_sons[0], second_itf_father_side, second_itf_son_side);
	second_sons[0]->set_father(father, second_itf_son_side, second_itf_father_side);
      }
    }
  }
}

/* fonction pour envoyer un message a un module */
void Send_Message(std::string text, uint8_t id, Module* source)
{
  uint8_t length = text.length();
  uint8_t message[NB_MAX_SBMSG][BUFFSIZE] = {0};
  int write_msg_i = 0;
  int write_byte_i = 1;
  int write_offset = 6;
  // segment routing
  for (uint8_t itf : segment_routing[id])
  {
    message[write_msg_i][write_byte_i] += itf << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
  }
  message[write_msg_i][write_byte_i] += END_SEGMENT_ROUTING << write_offset;
  incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);

  // message length
  uint8_t and_op = 0b11000000;
  for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
  {
    message[write_msg_i][write_byte_i] += ((length&and_op) >> read_offset) << write_offset;
    incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
    and_op = and_op >> 2;
  }
  // message
  for (int i = 0 ; i < length ; i++)
  {
    uint8_t and_op = 0b11000000;
    for (int read_offset = 6; read_offset >= 0; read_offset -= 2)
    {
      message[write_msg_i][write_byte_i] += ((((uint8_t) text[i])&and_op) >> read_offset) << write_offset;
      incr_indexes(&write_msg_i, &write_byte_i, &write_offset, NULL, 2);
      and_op = and_op >> 2;
    }
  }

  // header & send
  uint8_t nb_msg = write_msg_i + 1;
  for (int msg_i = 0; msg_i <= write_msg_i ; msg_i++)
  {
    message[msg_i][0] = (MSG_TO_MODULE << 6) + nb_msg;
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
    source->Send_Message(entry_itf, message[msg_i]);
  }

}

/* fonction pour lire l'init_r */
void Read_init_r(std::string init_r)
{
  std::vector<uint8_t> stack;
  int i = 0;
  int module_itf = 1;
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
      segment_routing[module_itf] = stack;
      module_itf++;
    }
    i++;
    value = ((int8_t) init_r[i]) - 48;
  }
}

void Identification_Process(Module* source)
{
  if (id_process_started && id_process_id < NB_MODULES)
  {
    std::string text = "id=" + std::to_string(id_process_id);
    Send_Message(text, id_process_id, source);
    id_process_id++;
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
      entry_itf = modules[0].Send_init();
      std::cout << "init sent!" << std::endl;
    }
    else if (aswr == "read init_r")
    {
      std::ifstream file;
      file.open("init_r.txt");
      std::string init_r;
      file >> init_r;
      if (init_r == "")
      {
	if (entry_itf == UNKNOWN_ITF)
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
	std::cout << "init_r sucessfully read, identification process initialized, you will soon be able to send message!" << std::endl;
      }
      file.close();
      id_process_started = true;
      Identification_Process(modules);
    }
    else if (aswr.find("send") == 0)
    {
      aswr = aswr.substr(aswr.find(" ")+1);
      int module_id = std::stoi(aswr.substr(0, aswr.find(" ")));
      std::string text  = aswr.substr(aswr.find(" ")+1);
      Send_Message(text, module_id, modules);
    }
    else if (aswr.find("deco") == 0)
    {
      aswr = aswr.substr(aswr.find(" ")+1);
      int module_id = std::stoi(aswr.substr(0, aswr.find(" ")));
      aswr = aswr.substr(aswr.find(" ")+1);
      uint8_t itf = std::stoi(aswr);
      for (int i = 0 ; i < NB_MODULES; i++)
      {
	if (modules[i].id == module_id)
	{
	  modules[i].deco_itf(itf);
	}
      }
    }
    else
    {
      Identification_Process(modules);
      for (int i = 0; i < NB_MODULES ; i++)
      {
	modules[i].Handle_All_Message();
      }
    }
  }
}
