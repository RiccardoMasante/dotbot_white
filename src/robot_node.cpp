#include "ros/ros.h"

#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
//#include "dotbot_msgs"
#include <sstream>

// ============= Attacco.hpp ================

class Attacco{
	private:
		const std::string _name;
		const int _danno;
		const int _atk_modifier;
		const int _def_modifier;
		const bool _priority;
		const std::string _type;
		const int _cure;
 	
	public:
		Attacco(std::string name , int danno , int atk_modifier , int def_modifier,
		 bool priority , std::string type , int cure);
  		~Attacco();
		
		//TODO ricevi colpo
		std::string get_name() const;
		int get_danno() const;
		int get_atk_mod() const;
		int get_def_mod() const;
		bool get_priority() const;
		std::string get_type() const;
		int get_cure() const;
	
};


// ============= Robot.hpp ================
class Robot{
	private:
  		ros::NodeHandle _nh;
		
		int _health_points;
		int _attack_points;
		int _defense_points;
		const int _speed_points;
		const std::string _type;
		Attacco * _attacco[4];  // array da 0 a 3
		int _attacco_da_sferrare; //valore da 1 a 4
		bool _newturn_flag;
	
		ros::Publisher hp_pub;
 		ros::Publisher log_pub;
		ros::Subscriber newturn_sub;
		ros::Subscriber get_move_sub;
		ros::Publisher myspeed_pub;
		
		void new_turn(const std_msgs::Empty & msg);
		void get_move(const std_msgs::UInt8 & numero_mossa);
		void send_log_msg(std::string testo_messaggio, int valore);
 	
	public:
  		Robot();
  		~Robot();
  		void run();

  		int get_hp() const;
  		void set_hp(int);
  		void inc_hp(int);

		int get_ap() const;
		void set_ap(int);
		void inc_ap(int);

		int get_dp() const;
		void set_dp(int);
		void inc_dp(int);

		int get_speed() const;

		std::string get_type() const;
		
		bool get_newturn_flag() const;
		void set_newturn_flag(bool);

		Attacco * get_attacco(int numero_attacco) const; //numero_attacco va da 1 a 4
		int get_attacco_da_sferrare() const;
		void set_attacco_da_sferrare(int ); //prende intero da 1 a 4, oppure un numero negativo se non ci sono mosse...

};


// ============= Robot.cpp ================


Robot::Robot() : _type("fire"), _speed_points(20)
{
	this->set_ap(20);
	this->set_dp(20);

	this->_attacco[0]=new Attacco("Fuoco Bomba" , 20 , 3 , 0 , false , "fire" , 0);
	this->_attacco[1]=new Attacco("Giga-Assorbimento" , 15 , 0 , 3 , false , "water" , 15);
	this->_attacco[2]=new Attacco("Terremoto" , 25 , 0 , 0 , false , "grass" , 0);
	this->_attacco[3]=new Attacco("Pugno Rapido" , 15 , 5 , -2 , true , "fire" , 0);

	this->set_attacco_da_sferrare(-1);

	this->log_pub =this->_nh.advertise<std_msgs::String>("log",1000);
	
	this->hp_pub = this->_nh.advertise<std_msgs::UInt8>("hp", 1000);
	this->set_hp(100);


	this->newturn_sub=this->_nh.subscribe("/newturn",1000, &Robot::new_turn,this);
	this->set_newturn_flag(false);

	this->get_move_sub=this->_nh.subscribe("move",1000, &Robot::get_move,this);
	
	this->myspeed_pub = this->_nh.advertise<std_msgs::UInt8>("speed",1000);

	this->set_newturn_flag(false);
	
}

Robot::~Robot() {
  	std::cout << "Nodo Terminato!" << std::endl;
}


void Robot::run() {
	ros::Rate loop_rate(1);
	while (ros::ok() && get_hp() > 0) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Robot::send_log_msg (std::string testo_messaggio, int valore) {
	std_msgs::String log_msg;
	std::stringstream ss;
	ss << testo_messaggio << valore;
	log_msg.data = ss.str();
	this->log_pub.publish(log_msg);
}
	
int Robot::get_hp() const
{ 
	return this->_health_points;
}

void Robot::set_hp(int hp) {
  	this->_health_points = hp;
	std_msgs::UInt8 hp_msg;
 	hp_msg.data = get_hp();
  	this->hp_pub.publish(hp_msg);

	this->send_log_msg( "Aggiornati i punti vita a ", get_hp());
}

void Robot::inc_hp(int increment) {
  	this->set_hp(this->get_hp() + increment);
}


int Robot::get_ap() const
{
	return this->_attack_points;
}

void Robot::set_ap(int ap) {
	this->_attack_points = ap;
}

void Robot::inc_ap(int increment) {
  	this->set_ap(this->get_ap() + increment);
}

int Robot::get_dp() const
{
	return this->_defense_points;
}

void Robot::set_dp(int dp) {
	this->_defense_points = dp;
}

void Robot::inc_dp(int increment) {
  	this->set_dp(this->get_dp() + increment);
}

int Robot::get_speed() const
{
	return this->_speed_points;
}

std::string Robot::get_type() const
{
	return this->_type;
}

Attacco * Robot::get_attacco(int numero_attacco) const
{
	if (numero_attacco > 0 ) {
		return this->_attacco[numero_attacco-1];
	} else {
		return NULL;
	}
}

void Robot::set_attacco_da_sferrare(int numero_attacco) 
{
	if (numero_attacco <= 4 && numero_attacco >= 1) { 
		this->_attacco_da_sferrare = numero_attacco;
	} else {
		this->_attacco_da_sferrare = -1;
	}
}

int Robot::get_attacco_da_sferrare() const
{
	return this->_attacco_da_sferrare;
}

bool Robot::get_newturn_flag() const
{
	return this->_newturn_flag;
}

void Robot::set_newturn_flag(bool flag)
{	
	this->_newturn_flag = flag;	
}

void Robot::new_turn(const std_msgs::Empty & msg)
{
	set_newturn_flag(true);
}

void Robot::get_move(const std_msgs::UInt8 & numero_mossa)
{
	if (this->get_newturn_flag())
	{
		this->set_attacco_da_sferrare(numero_mossa.data);
		this->set_newturn_flag(false);

		std_msgs::UInt8 speed_msg;
		speed_msg.data = this->get_speed() + 100 * (int) get_attacco(get_attacco_da_sferrare())->get_priority();
		myspeed_pub.publish(speed_msg);
		
		send_log_msg("ho scelto il mio attacco e comunicata la mia velocita... ti distruggo!",0);
	}	

}




// ============= Attacco.cpp ================

Attacco::Attacco(std::string name , int danno , int atk_modifier , int def_modifier,
		 bool priority , std::string type , int cure) :
		 _name(name), 
		 _danno(danno), 
		 _atk_modifier(atk_modifier), 
		 _def_modifier(def_modifier),
		 _priority(priority), 
		 _type(type), 
		 _cure(cure)
{
	//
}


Attacco::~Attacco() 
{

}

std::string Attacco::get_name() const
{
	return this->_name;
}

int Attacco::get_danno() const
{
	return this->_danno;
}

int Attacco::get_atk_mod() const
{
	return this->_atk_modifier;
}
	
int Attacco::get_def_mod() const
{
	return this->_def_modifier;
}
	
bool Attacco::get_priority() const
{
	return this->_priority;
}

std::string Attacco::get_type() const
{
	return this->_type;
}
	
int Attacco::get_cure() const
{
	return this->_cure;
}

int main(int argc, char **argv)
{	
	//inizializzo il nodo
	ros::init(argc, argv, "white_bot");
	
	Robot white;
	white.run();
  
  	return 0;
}
