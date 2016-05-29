#include "ros/ros.h"

#include "robot_white/get_moves_srv.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "dotbot_msgs"
#include <sstream>

// ============= Attack.hpp ================

class Attack{
	private:
		const std::string _name;
		const int _damage;
		const int _atk_modifier;
		const int _def_modifier;
		const bool _priority;
		const std::string _type;
		const int _cure;
 	
	public:
		Attack(std::string name , int damage , int atk_modifier , int def_modifier,
		 bool priority , std::string type , int cure);
  		~Attack();
		
		std::string getName() const;
		int getDamage() const;
		int getAtkMod() const;
		int getDefMod() const;
		bool getPriority() const;
		std::string getType() const;
		int getCure() const;
	
};


// ============= Attack.cpp ================

Attack::Attack(std::string name , int damage , int atk_modifier , int def_modifier,
		 bool priority , std::string type , int cure) :
		 _name(name), 
		 _damage(damage), 
		 _atk_modifier(atk_modifier), 
		 _def_modifier(def_modifier),
		 _priority(priority), 
		 _type(type), 
		 _cure(cure)
{
	//
}


Attack::~Attack() 
{

}

std::string Attack::getName() const
{
	return this->_name;
}

int Attack::getDamage() const
{
	return this->_damage;
}

int Attack::getAtkMod() const
{
	return this->_atk_modifier;
}
	
int Attack::getDefMod() const
{
	return this->_def_modifier;
}
	
bool Attack::getPriority() const
{
	return this->_priority;
}

std::string Attack::getType() const
{
	return this->_type;
}
	
int Attack::getCure() const
{
	return this->_cure;
}


// ============= Robot.hpp ================
class Robot{
	private:

  		ros::NodeHandle _nh;
		
		int _health_points;
		int _attack_points;
		int _defense_points;
		const int _speed_points;
		const std::string _type;
		Attack * _attack[4];  // array da 0 a 3
		int _attack_to_launch; //valore da 1 a 4
		bool _newturn_flag;
	
		ros::Publisher hp_pub;
 		ros::Publisher log_pub;
		ros::Subscriber newturn_sub;
		ros::Subscriber get_move_sub;
		ros::Publisher myspeed_pub;
		ros::ServiceServer moves_srv;
		ros::Subscriber myturn_sub;
		ros::Publisher atk_pub;

		void newTurn(const std_msgs::Empty & msg);
		void getMove(const std_msgs::UInt8 & move_number);
		void sendLogMsg(std::string testo_messaggio, int valore);
		bool movesNamesCb(robot_white::get_moves_srv::Request &req,
			robot_white::get_moves_srv::Response &res);
		int calcoloDanno();
 	
	public:
  		Robot();
  		~Robot();
  		void run();

  		int getHp() const;
  		void setHp(int);
  		void incHp(int);

		int getAp() const;
		void setAp(int);
		void incAp(int);

		int getDp() const;
		void setDp(int);
		void incDp(int);

		int getSpeed() const;

		std::string getType() const;
		
		bool getNewturnFlag() const;
		void setNewturnFlag(bool);

		Attack * getAttack(int attack_number) const; //attack_number va da 1 a 4
		int getAttackToLaunch() const;
		void setAttackToLaunch(int ); //prende intero da 1 a 4, oppure un numero negativo se non ci sono mosse...

};


// ============= Robot.cpp ================


Robot::Robot() : _type("fire"), _speed_points(20)
{
	this->setAp(20);
	this->setDp(20);

	this->_attack[0]=new Attack("Fuoco Bomba" , 20 , 3 , 0 , false , "fire" , 0);
	this->_attack[1]=new Attack("Giga-Assorbimento" , 15 , 0 , 3 , false , "water" , 15);
	this->_attack[2]=new Attack("Terremoto" , 25 , 0 , 0 , false , "grass" , 0);
	this->_attack[3]=new Attack("Pugno Rapido" , 15 , 5 , -2 , true , "fire" , 0);

	this->setAttackToLaunch(-1);

	this->log_pub =this->_nh.advertise<std_msgs::String>("log",1000);
	
	this->hp_pub = this->_nh.advertise<std_msgs::UInt8>("hp", 1000);
	this->setHp(100);


	this->newturn_sub=this->_nh.subscribe("/newturn",1000, &Robot::newTurn,this);
	this->setNewturnFlag(false);

	this->get_move_sub=this->_nh.subscribe("move",1000, &Robot::getMove,this);
	
	this->myspeed_pub = this->_nh.advertise<std_msgs::UInt8>("speed",1000);

	this->moves_srv = this->_nh.advertiseService("get_moves_srv",&Robot::movesNamesCb,this);

	this->setNewturnFlag(false);
	
	this->myturn_sub = this->_nh.subscribe("white_turn",1000, &Robot::attack,this);
	this->atk_pub = this->_nh.advertise<dotbot_msgs::Attacco>("/silver/danno",1000);
	
}

Robot::~Robot() {
  	std::cout << "Nodo Terminato!" << std::endl;
}


void Robot::run() {

	ros::Rate loop_rate(1);
	while (ros::ok() && getHp() > 0) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Robot::sendLogMsg (std::string testo_messaggio, int valore) {
	std_msgs::String log_msg;
	std::stringstream ss;
	ss << testo_messaggio << valore;
	log_msg.data = ss.str();
	this->log_pub.publish(log_msg);
}
	
int Robot::getHp() const
{ 
	return this->_health_points;
}

void Robot::setHp(int hp) {
  	this->_health_points = hp;
	std_msgs::UInt8 hp_msg;
 	hp_msg.data = getHp();
  	this->hp_pub.publish(hp_msg);

	this->sendLogMsg( "Aggiornati i punti vita a ", getHp());
}

void Robot::incHp(int increment) {
  	this->setHp(this->getHp() + increment);
}


int Robot::getAp() const
{
	return this->_attack_points;
}

void Robot::setAp(int ap) {
	this->_attack_points = ap;
}

void Robot::incAp(int increment) {
  	this->setAp(this->getAp() + increment);
}

int Robot::getDp() const
{
	return this->_defense_points;
}

void Robot::setDp(int dp) {
	this->_defense_points = dp;
}

void Robot::incDp(int increment) {
  	this->setDp(this->getDp() + increment);
}

int Robot::getSpeed() const
{
	return this->_speed_points;
}

std::string Robot::getType() const
{
	return this->_type;
}

Attack * Robot::getAttack(int attack_number) const
{
	if (attack_number > 0 ) {
		return this->_attack[attack_number-1];
	} else {
		return NULL;
	}
}

void Robot::setAttackToLaunch(int attack_number) 
{
	if (attack_number <= 4 && attack_number >= 1) { 
		this->_attack_to_launch = attack_number;
	} else {
		this->_attack_to_launch = -1;
	}
}

int Robot::getAttackToLaunch() const
{
	return this->_attack_to_launch;
}

bool Robot::getNewturnFlag() const
{
	return this->_newturn_flag;
}

void Robot::setNewturnFlag(bool flag)
{	
	this->_newturn_flag = flag;	
}

void Robot::newTurn(const std_msgs::Empty & msg)
{
	setNewturnFlag(true);
}

void Robot::getMove(const std_msgs::UInt8 & move_number)
{
	if (this->getNewturnFlag())
	{
		this->setAttackToLaunch(move_number.data);
		this->setNewturnFlag(false);

		std_msgs::UInt8 speed_msg;
		speed_msg.data = this->getSpeed() + 100 * (int) getAttack(getAttackToLaunch())->getPriority();
		myspeed_pub.publish(speed_msg);
		
		sendLogMsg("ho scelto il mio attack e comunicata la mia velocita... ti distruggo!",0);
	}	

}

bool Robot::movesNamesCb(robot_white::get_moves_srv::Request &req,
		robot_white::get_moves_srv::Response &res)
{
	res.move1.data = getAttack(1)->getName();
	res.move2.data = getAttack(2)->getName();
	res.move3.data = getAttack(3)->getName();
	res.move4.data = getAttack(4)->getName();
	return true;
}

void Robot::attack(const std_msgs::Empty & msg)
{
	dotbot_msg::attacco atk_msg;
	Attacco *attack_tmp=this->getAttack(this->getAttackToLaunch());
	
	//calcolo danno
	atk_msg.danno.data=this->calcoloDanno();
	atk_msg.type.data=attack_tmp->getType();
	
	//comunico atk_msg 
	this->atk_pub.publish(atk_msg);

	//aggiungere movimenti

	//aggiorno miei valori
	this->incHp(attack_tmp->getCure());
	this->incAp(attack_tmp->getAtkMod());
	this->incDp(attack_tmp->getDefMod());
	
	//scrivo nel log	
	sendLogMsg("ho attaccatoooo",0);
}

int Robot::calcoloDanno()
{
	float a;
	a=(this->getAttack(this->getAttackToLaunch())->getDanno())*(this->getAp())*RandomFloat(0.8,1.2);
	return ceil(a);
}


// ============== main.cpp ================

int main(int argc, char **argv)
{	
	//inizializzo il nodo
	ros::init(argc, argv, "white_bot");
	
	Robot white;
	white.run();
  
  	return 0;
}
